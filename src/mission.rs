use core::{
    cell::RefCell,
    fmt::{self, Write},
};

use crate::{
    futures::{NbFuture, YieldFuture},
    radio,
    usb_serial::get_serial,
};
use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use embedded_hal::blocking::i2c;
use futures::join;
use heapless::Vec;
use libm::pow;
use phf::phf_map;
use stm32f4xx_hal::{
    prelude::_fugit_DurationExtU32,
    timer::{self, CounterMs},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Designator {
    A,
    B,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NodeType {
    CanSat(Designator),
    GroundStation(Designator),
    Rocket,
}

const NODE_TYPE: NodeType = NodeType::GroundStation(Designator::A);
const NODES: phf::Map<u64, NodeType> = phf_map! {
    0x0013A20041EFD4CCu64 => NodeType::CanSat(Designator::A),
    0x0013A20041EFD4D0u64 => NodeType::GroundStation(Designator::A),
    0x0013A20041EFD44Au64 => NodeType::Rocket,
};

const PEER_LEN: usize = 2;

pub struct MissionState {}

impl MissionState {
    pub fn new() -> Self {
        MissionState {}
    }

    pub async fn handle_radio_communications(self: &Self) {
        loop {
            let frame = radio::try_claim_frame(|frame| match frame {
                radio::APIFrame::ReceivePacket {
                    message: Some(message),
                    ..
                } => match message {
                    radio::Message::Ping(_) => true,
                    radio::Message::Pong(_) => false,
                    radio::Message::Gps { lat, lon, alt } => {
                        matches!(NODE_TYPE, NodeType::GroundStation(_))
                    }
                    radio::Message::PressureTemp { pressure, temp } => {
                        matches!(NODE_TYPE, NodeType::GroundStation(_))
                    }
                },
                _ => false,
            });
            if let Some(radio::APIFrame::ReceivePacket {
                payload,
                source_address,
                message: Some(message),
                ..
            }) = frame
            {
                match message {
                    radio::Message::Ping(id) => {
                        hprintln!("Received ping from {}", source_address);
                        let reply = radio::Message::Pong(id);
                        let tx_req = radio::APIFrame::TransmitRequest {
                            frame_id: radio::next_id(),
                            destination_address: source_address,
                            broadcast_radius: 0,
                            transmit_options: 0,
                            data: reply.data(),
                        };
                        let mut buf = [0; 256];
                        tx_req.tx(&mut buf).await;
                        tx_req.await_response().await;
                    }
                    radio::Message::Gps { lat, lon, alt } => {
                        hprintln!("Received GPS from {}", source_address);
                        hprintln!("{:?}", (lat, lon, alt));
                    }
                    radio::Message::PressureTemp { pressure, temp } => {
                        write!(get_serial(), "prtp,{:016x},{},{}\n", source_address, pressure, temp).unwrap();
                    }
                    _ => unreachable!(),
                };
            }
            YieldFuture::new().await;
        }
    }

    pub async fn start<I2C, TIM>(self: &mut Self, bmp: bmp388::BMP388<I2C>, timer: CounterMs<TIM>)
    where
        I2C: i2c::WriteRead,
        I2C::Error: fmt::Debug,
        TIM: timer::Instance,
    {
        let long_running_tasks = async {
            let f1 = self.handle_radio_communications();

            let f2 = async {
                if matches!(NODE_TYPE, NodeType::GroundStation(_)) {
                    self.handle_usb_communication().await;
                }
            };

            let f3 = async {
                if matches!(NODE_TYPE, NodeType::GroundStation(_)) {
                    self.report_sensor_data(bmp, timer).await;
                }
            };
            join!(f1, f2, f3)
        };

        futures::future::join(long_running_tasks, self.start_mission()).await;
    }

    async fn start_mission(&self) {
        hprintln!("Starting mission");
        self.init_all_subsystems().await;
    }

    async fn init_all_subsystems(&self) {
        let f1 = self.discover_network();
        let f2 = async {
            if !matches!(NODE_TYPE, NodeType::GroundStation(_)) {
                self.init_gps().await;
            }
        };
        join!(f1, f2);
    }

    async fn handle_usb_communication(&self) {
        hprintln!("Initializing USB communication");
        let mut buffer = [0u8; 256];
        let mut bytes = get_serial().read(&mut buffer).await.unwrap();
        loop {
            if bytes > 0 {
                if buffer.starts_with(b"begin") {
                    hprintln!("USB begin");
                    get_serial().write(b"ack\n").await;
                }
            }
            bytes = get_serial().read(&mut buffer).await.unwrap();
        }
    }

    async fn discover_network(self: &Self) {
        self.radio_discover().await;
        self.radio_ping_all().await;
    }

    async fn init_gps(self: &Self) {}

    async fn radio_discover(self: &Self) {
        // Discover radios on network
        let buf = &mut [0u8; 32];
        let mut peers_found = 0;
        let mut command_successful = false;
        let api_frame =
            radio::APIFrame::local_command_request(radio::next_id(), *b"ND", [0u8; 256], 0);
        loop {
            if !command_successful {
                api_frame.tx(buf).await;
            }

            let response = api_frame.await_response().await;
            match response {
                radio::APIFrame::LocalCommandResponse { success, data, .. } => {
                    command_successful = success == 0;
                    if success == 0 {
                        match data {
                            radio::LocalCommandResponseData::NetworkDiscoverResponse {
                                serial,
                                identifier,
                                ..
                            } => {
                                hprintln!("Peer {:x} found. Identifier: {}", serial, identifier);
                                peers_found += 1;
                                if peers_found == PEER_LEN {
                                    break;
                                }
                            }
                            _ => unreachable!(),
                        }
                    } else {
                        hprintln!("Discovery failed. Retrying.");
                    }
                }
                _ => unreachable!(),
            }
        }
    }

    async fn radio_ping_all(&self) {
        // Unsure how to parallelize this.
        for &peer in NODES.keys() {
            if NODES[&peer] == NODE_TYPE {
                continue;
            }
            hprintln!("Pinging {:x}", peer);
            let mut buf = Vec::new();
            buf.extend_from_slice(b"PING").unwrap();
            let id = radio::next_id() as u8;
            buf.push(id).unwrap();
            let api_frame = radio::APIFrame::TransmitRequest {
                frame_id: radio::next_id(),
                destination_address: peer,
                broadcast_radius: 0,
                transmit_options: 0,
                data: buf,
            };
            let mut buf2 = [0u8; 512];
            api_frame.tx(&mut buf2).await;
            radio::claim_frame(|frame| match frame {
                radio::APIFrame::ReceivePacket {
                    source_address,
                    payload,
                    message,
                    ..
                } => match message {
                    Some(radio::Message::Pong(x)) => {
                        if *source_address == peer && *x == id {
                            hprintln!("Pong from {:x}", source_address);
                            true
                        } else {
                            false
                        }
                    }
                    _ => false,
                },
                _ => false,
            })
            .await;
        }

        hprintln!("All pings ponged");
    }
    // async fn progress_radio(self: &mut Self) {
    //     match self.radio_state {
    //         RadioState::NotStarted => {
    //             self.radio_discover().await;
    //             self.radio_state = RadioState::AwaitingPeers(1);
    //         }
    //         RadioState::AwaitingPeers(n) => loop {
    //             let mut buf = [0u8; 128];
    //             let bytes = radio::rx(&mut buf).await;
    //             if bytes == 0 {
    //                 continue;
    //             }
    //             //hprintln!("{:?}", &buf[0..bytes]);
    //             let api_frame = radio::APIFrame::parse(&buf[..bytes]).unwrap_or_else(|| {
    //                 hprintln!("invalid API frame {:x?}", &buf[..bytes]);
    //                 panic!()
    //             });
    //             match api_frame {
    //                 radio::APIFrame::LocalCommandResponse {
    //                     data:
    //                         radio::LocalCommandResponseData::NetworkDiscoverResponse {
    //                             identifier,
    //                             identifier_len,
    //                             serial,
    //                             ..
    //                         },
    //                     ..
    //                 } => {
    //                     hprintln!(
    //                         "Found peer {}",
    //                         core::str::from_utf8(&identifier[..identifier_len])
    //                             .expect("Non UTF-8 identifier")
    //                     );
    //                     if n == 1 {
    //                         self.peers[self.peers.len() - n as usize] = serial;
    //                         self.radio_state = RadioState::Diagnostics;
    //                         break;
    //                     } else {
    //                         self.radio_state = RadioState::AwaitingPeers(n - 1);
    //                         break;
    //                     }
    //                 }
    //                 _ => {
    //                     hprintln!("Unhandled frame: {:?}", api_frame);
    //                 }
    //             }
    //         },
    //         RadioState::Diagnostics => {
    //             hprintln!("Sending ping to peer");
    //             let mut data = [0u8; 32];
    //             data[..11].copy_from_slice(b"Hello World");
    //             let data_length = 11;
    //             let api_frame = radio::APIFrame::TransmitRequest {
    //                 frame_id: 0x69,
    //                 destination_address: self.peers[0],
    //                 broadcast_radius: 0,
    //                 transmit_options: 0,
    //                 data,
    //                 data_length,
    //             };
    //             let mut buf = [0u8; 128];
    //             api_frame.tx(&mut buf).await;
    //         }
    //     }
    // }

    async fn report_sensor_data<I2C, TIM>(
        &self,
        mut bmp: bmp388::BMP388<I2C>,
        mut timer: CounterMs<TIM>,
    ) where
        TIM: timer::Instance,
        I2C: embedded_hal::blocking::i2c::WriteRead,
        I2C::Error: core::fmt::Debug,
    {
        timer.start(5000.millis()).unwrap();
        loop {
            let values = bmp.sensor_values().unwrap();

            let alt = (1.0 / 0.0065)
                * (pow((values.pressure) / 101325.0, 1.0 / 5.257) - 1.0)
                * values.temperature;

            /*hprintln!(
                "Temperature: {:.2}°C, Pressure: {:.2}Pa, Altitude: {:.2}m",
                values.temperature,
                values.pressure,
                alt
            );*/

            let ground_stations = NODES
                .keys()
                .filter(|x| matches!(NODES[x], NodeType::GroundStation(_)));

            let data = radio::Message::PressureTemp {
                pressure: values.pressure,
                temp: values.temperature,
            }
            .data();

            for gs in ground_stations {
                let frame = radio::APIFrame::TransmitRequest {
                    frame_id: radio::next_id(),
                    destination_address: *gs,
                    broadcast_radius: 0,
                    transmit_options: 0,
                    data: data.clone(),
                };

                let mut buf = [0u8; 128];
                frame.tx(&mut buf).await;
                frame.await_response().await; // for now discard response
            }
            NbFuture::new(|| timer.wait()).await.unwrap();
        }
    }
}

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use futures::join;

use crate::{radio, futures::YieldFuture};

const PEER_LEN: usize = 1;

pub struct MissionState {
    radio_state: RadioState,
    // FIXME: come up with a better lifetime so we don't have to use a Mutex<RefCell<T>>
    peers: Mutex<RefCell<[u64; PEER_LEN]>>,
}

enum RadioState {
    NotStarted,
    AwaitingPeers(u8),
    Diagnostics,
}

impl MissionState {
    pub fn new() -> Self {
        MissionState {
            radio_state: RadioState::NotStarted,
            peers: Mutex::new(RefCell::new([0; PEER_LEN])),
        }
    }

    pub async fn respond_to_pings() {
        loop {
            let frame = radio::try_claim_frame(|frame| match frame {
                radio::APIFrame::ReceivePacket { payload,  .. } => &payload[0..4] == b"PING",
                _ => false,
            });
            if let Some(radio::APIFrame::ReceivePacket {
                payload,
                source_address,
                ..
            }) = frame
            {
                hprintln!("Received ping from {}", source_address);
                let mut data = [0; 256];
                data[0..4].copy_from_slice(b"PONG");
                data[4] = payload[4];

                let tx_req = radio::APIFrame::TransmitRequest {
                    frame_id: radio::next_id(),
                    destination_address: source_address,
                    broadcast_radius: 0,
                    transmit_options: 0,
                    data,
                    data_length: 5,
                };
                let mut buf = [0; 256];
                tx_req.tx(&mut buf).await;
            }
            YieldFuture::new().await;
        }
    }

    pub async fn start(self: &mut Self) {
        hprintln!("Starting mission");
        let f0 = radio::parse_recvd_data();
        let f1 = Self::respond_to_pings();
        let f2 = async {
            self.radio_discover().await;
            self.radio_ping_all().await;
        };
        let f3 = self.init_gps();

        join!(f0, f1, f2, f3);
    }

    async fn init_gps(self: &Self) {}

    async fn radio_discover(self: &Self) {
        hprintln!("Discovering radio");
        // Discover radios on network
        let buf = &mut [0u8; 32];
        let mut peers_found = 0;
        let mut command_successful = false;
        let api_frame =
            radio::APIFrame::local_command_request(radio::next_id(), *b"ND", [0u8; 256], 0);
        hprintln!("Sending ND command");
        loop {
            if !command_successful {
                api_frame.tx(buf).await;
            }
            hprintln!("Waiting for response");

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
                                cortex_m::interrupt::free(|cs| {
                                    self.peers.borrow(cs).borrow_mut()[peers_found] = serial;
                                    peers_found += 1;
                                });
                                hprintln!("Peer {:x} found. Identifier: {}", serial, identifier);
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
        hprintln!("Pinging all peers");
        let peers = cortex_m::interrupt::free(|cs| self.peers.borrow(cs).borrow().clone());

        for peer in peers {
            let mut buf = [0u8; 256];
            buf[0..4].copy_from_slice(b"PING");
            buf[4] = radio::next_id() as u8;
            let api_frame = radio::APIFrame::TransmitRequest {
                frame_id: radio::next_id(),
                destination_address: peer,
                broadcast_radius: 0,
                transmit_options: 0,
                data: buf,
                data_length: 5,
            };
            let mut buf2 = [0u8; 512];
            api_frame.tx(&mut buf2).await;
            radio::claim_frame(|frame| {
                match frame {
                    radio::APIFrame::ReceivePacket { source_address, payload, .. } => {
                        if *source_address == peer && &payload[0..4] == b"PONG" && payload[4] == buf[4] {
                            hprintln!("Pong from {:x}", source_address);
                            true
                        } else { return false }
                    }
                    _ => false,
                }
            }).await;
        }
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
}

use cortex_m_semihosting::hprintln;
use futures::join;

use crate::radio;

pub struct MissionState {
    radio_state: RadioState,
    peers: [u64; 2],
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
            peers: [0; 2],
        }
    }

    pub async fn respond_to_pings() {
        loop {
            let frame = radio::try_claim_frame(|frame| match frame {
                radio::APIFrame::ReceivePacket { .. } => true,
                _ => false,
            });
            if let Some(radio::APIFrame::ReceivePacket {
                mut payload,
                source_address,
                ..
            }) = frame
            {
                todo!("respond to ping");
            }
        }
    }

    pub async fn start(self: &mut Self) {
        let f1 = Self::respond_to_pings();
        let f2 = async {
            self.radio_discover().await;
            //self.radio_ping_all().await;
        };
        let f3 = self.init_gps();

        join!(f1, f2, f3);
    }

    async fn init_gps(self: &Self) {
        
    }

    async fn radio_discover(self: &Self) {
        // Discover radios on network
        let buf = &mut [0u8; 32];
        let mut peers_found = 0;
        let mut command_successful = false;
        let api_frame = radio::APIFrame::local_command_request(99, *b"ND", [0u8; 32], 0);
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
                                self.peers[peers_found] = serial;
                                peers_found += 1;
                                hprintln!("Peer {:x} found. Identifier: {}", serial, identifier);
                                if peers_found == 2 {
                                    break;
                                }
                            }
                            _ => unreachable!(),
                        }
                    }
                }
                _ => unreachable!(),
            }
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

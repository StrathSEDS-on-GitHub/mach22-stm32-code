use cortex_m_semihosting::hprintln;

use crate::radio;

pub struct MissionState {
    radio_state: RadioState,
    peers: [u64; 1],
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
            peers: [0; 1],
        }
    }

    pub async fn start(self: &mut Self) {
        let mut buffer = [0u8; 32];
        let mut i = 0;
        loop {
            self.progress_radio().await;
        }
    }

    async fn radio_discover(self: &mut Self) {
        // Discover radios on network
        let buf = &mut [0u8; 32];
        let api_frame = radio::APIFrame::local_command_request(99, *b"ND", [0u8; 32], 0);
        api_frame.tx(buf).await;
    }

    // Progresses the radio state machine
    async fn progress_radio(self: &mut Self) {
        match self.radio_state {
            RadioState::NotStarted => {
                self.radio_discover().await;
                self.radio_state = RadioState::AwaitingPeers(1);
            }
            RadioState::AwaitingPeers(n) => loop {
                let mut buf = [0u8; 128];
                let bytes = radio::rx(&mut buf).await;
                if bytes == 0 {
                    continue;
                }
                //hprintln!("{:?}", &buf[0..bytes]);
                let api_frame = radio::APIFrame::parse(&buf[..bytes]).unwrap_or_else(|| { hprintln!("invalid API frame {:x?}", &buf[..bytes]); panic!() });
                match api_frame {
                    radio::APIFrame::LocalCommandResponse {
                        data:
                            radio::LocalCommandResponseData::NetworkDiscoverResponse {
                                identifier,
                                identifier_len,
                                serial,
                                ..
                            },
                        ..
                    } => {
                        hprintln!(
                            "Found peer {}",
                            core::str::from_utf8(&identifier[..identifier_len])
                                .expect("Non UTF-8 identifier")
                        );
                        if n == 1 {
                            self.peers[self.peers.len() - n as usize] = serial;
                            self.radio_state = RadioState::Diagnostics;
                            break;
                        } else {
                            self.radio_state = RadioState::AwaitingPeers(n - 1);
                            break;
                        }
                    }
                    _ => {
                        hprintln!("Unhandled frame: {:?}", api_frame);
                    }
                }
            },
            RadioState::Diagnostics => {
                hprintln!("Sending ping to peer");
                let mut data = [0u8; 32];
                data[..11].copy_from_slice(b"Hello World");
                let data_length = 11;
                let api_frame = radio::APIFrame::TransmitRequest {
                    frame_id: 0x69,
                    destination_address: self.peers[0],
                    broadcast_radius: 0,
                    transmit_options: 0,
                    data,
                    data_length,
                };
                let mut buf = [0u8; 128];
                api_frame.tx(&mut buf).await;
            }
        }
    }
}

use core::{
    cell::{Cell, RefCell},
    fmt::{self, Write},
    sync::atomic::{AtomicBool, Ordering},
};

use crate::{
    futures::{NbFuture, YieldFuture},
    gps,
    radio::{self, APIFrame},
    sdcard::{self, get_logger, FixedWriter},
    usb_serial::get_serial,
};
use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use embedded_hal::blocking::i2c;
use futures::join;
use heapless::{Deque, Vec};
use libm::pow;
use phf::phf_map;
use stm32f4xx_hal::{
    prelude::_fugit_DurationExtU32,
    timer::{self, CounterMs},
};
use time::{macros::date, Date, Month, OffsetDateTime, PrimitiveDateTime, Time};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Designator {
    A,
    B,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeType {
    CanSat(Designator),
    GroundStation(Designator),
    Rocket,
}

pub const NODE_TYPE: NodeType = NodeType::Rocket;

const NODES: phf::Map<u64, NodeType> = phf_map! {
    //0x0013A20041EFD4CCu64 => NodeType::CanSat(Designator::A),
    0x0013A20041EFD4D0u64 => NodeType::GroundStation(Designator::A),
    0x0013A20041EFD44Au64 => NodeType::Rocket,
};

const PEER_LEN: usize = 1;
static GPS_HAS_FIX: AtomicBool = AtomicBool::new(false);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    PreLaunch,
    Ascent,
    Descent,
    Recovery,
}

pub struct MissionState {
    launch_t: Cell<i64>,
    stage: Cell<Stage>,
    alt_readings: Mutex<RefCell<Deque<f64, 15>>>,
}

impl MissionState {
    pub fn new() -> Self {
        MissionState {
            launch_t: Cell::new(0),
            stage: Cell::new(Stage::PreLaunch),
            alt_readings: Mutex::new(RefCell::new(Deque::new())),
        }
    }

    fn get_ground_stations() -> impl Iterator<Item = u64> {
        NODES
            .keys()
            .filter(|k| matches!(NODES[k], NodeType::GroundStation(_)))
            .map(|k| *k)
    }

    fn get_remote_nodes() -> impl Iterator<Item = u64> {
        NODES
            .keys()
            .filter(|k| !matches!(NODES[k], NodeType::GroundStation(_)))
            .map(|k| *k)
    }

    fn should_send_sensor_data(&self) -> bool {
        match NODE_TYPE {
            NodeType::Rocket => true,
            NodeType::CanSat(_) => match self.stage.get() {
                Stage::Descent | Stage::Recovery => true,
                _ => false,
            },
            NodeType::GroundStation(_) => false,
        }
    }

    pub async fn handle_radio_communications(self: &Self) {
        loop {
            let frame = radio::try_claim_frame(|frame| match frame {
                radio::APIFrame::ReceivePacket {
                    message: Some(message),
                    ..
                } => match message {
                    radio::Message::Ping(_) => true,
                    radio::Message::Gps { .. } => true,
                    radio::Message::PressureTemp { .. } => true,
                    radio::Message::SetLaunchT(_) => true,
                    _ => false,
                },
                _ => false,
            });
            if let Some(radio::APIFrame::ReceivePacket {
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
                        write!(
                            get_serial(),
                            "prtp,{:016x},{},{}\n",
                            source_address,
                            pressure,
                            temp
                        )
                        .unwrap();
                    }
                    radio::Message::SetLaunchT(t) => {
                        self.launch_t.replace(t);
                        if Self::is_ground_station() {
                            write!(get_serial(), "newt,{}\n", t).unwrap();
                        } else {
                            get_logger().log(format_args!("New launch time {}\n", t));
                        }
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
                if !matches!(NODE_TYPE, NodeType::GroundStation(_)) {
                    self.report_sensor_data(bmp, timer).await;
                }
            };

            let f4 = async {
                self.handle_gps_packets().await;
            };
            join!(f1, f2, f3, f4)
        };

        futures::future::join(long_running_tasks, self.start_mission()).await;
    }

    async fn start_mission(&self) {
        // PRELAUNCH STAGE
        self.init_all_subsystems().await;
        if !Self::is_ground_station() {
            self.send_ready_for_ascent().await;
        } else {
            self.wait_for_ready_messages().await;
            get_serial().write(b"reqt\n").await;
        }

        self.wait_for_launch_t().await;

        get_logger().log_str("Launch!");
        self.stage.replace(Stage::Ascent);

        // ASCENT STAGE
        self.wait_for_ascent().await;
        self.stage.replace(Stage::Descent);
    }

    async fn wait_for_ascent(&self) {
        loop {
            let mut readings = [0.0; 15];
            cortex_m::interrupt::free(|cs| {
                let alt_readings = self.alt_readings.borrow(cs).borrow();
                let mut i = 0;
                for item in alt_readings.iter() {
                    readings[i] = *item;
                    i += 1;
                }
            });
            let mut diffs = [0.0; 14];
            for i in 0..14 {
                diffs[i] = readings[i + 1] - readings[i];
            }
            let average_diff = diffs.iter().sum::<f64>() / 15.0;
            if average_diff < -0.5 {
                break;
            }
            YieldFuture::new().await;
        }
    }

    async fn wait_for_launch_t(&self) {
        loop {
            let (current_time, launch_t) = cortex_m::interrupt::free(|cs| {
                let mut rtc_ref = unsafe { crate::RTC.borrow(cs) }.borrow_mut();
                let rtc = rtc_ref.as_mut().unwrap();
                let current_time = rtc.get_datetime().assume_utc().unix_timestamp();
                let launch_t = self.launch_t.get();
                (current_time, launch_t)
            });

            //get_logger().log(format_args!("Current time: {}. Launch at {}", current_time, self.launch_t.get()));
            if current_time > launch_t && launch_t > 0 {
                break;
            }
            YieldFuture::new().await;
        }
    }

    async fn wait_for_ready_messages(&self) {
        let ascent_nodes = NODES
            .values()
            .filter(|it| !matches!(*it, NodeType::GroundStation(_)))
            .count();
        let mut ready_count = 0;
        loop {
            let frame = radio::try_claim_frame(|frame| match frame {
                radio::APIFrame::ReceivePacket {
                    message: Some(message),
                    ..
                } => match message {
                    radio::Message::ReadyForAscent => true,
                    _ => false,
                },
                _ => false,
            });
            if let Some(radio::APIFrame::ReceivePacket {
                source_address,
                message: Some(message),
                ..
            }) = frame
            {
                match message {
                    radio::Message::ReadyForAscent => {
                        get_logger().log(format_args!("ready from {:x}", source_address));
                        ready_count += 1;
                        if ready_count == ascent_nodes {
                            break;
                        }
                    }
                    _ => unreachable!(),
                };
            }
            YieldFuture::new().await;
        }
    }

    async fn init_all_subsystems(&self) {
        let f1 = self.discover_network();
        let f2 = async {
            if !Self::is_ground_station() {
                self.wait_for_gps().await;
            }
        };
        join!(f1, f2);
        get_logger().log(format_args!("All subsystems initialized"));
    }

    async fn wait_for_gps(&self) {
        hprintln!("Waiting for GPS");
        while !GPS_HAS_FIX.load(Ordering::SeqCst) {
            YieldFuture::new().await;
        }
    }

    async fn handle_usb_communication(&self) {
        hprintln!("Initializing USB communication");
        let mut buffer = [0u8; 256];
        let mut bytes = get_serial().read(&mut buffer).await.unwrap();
        loop {
            if bytes > 0 {
                for line in buffer[..bytes].split(|c| *c == b'\n') {
                    if line.starts_with(b"ping") {
                        get_serial().write(b"pong\n").await;
                    } else if line.starts_with(b"sett") {
                        let t = line.split(|t| *t == ',' as u8);
                        let t = core::str::from_utf8(t.skip(1).next().unwrap()).unwrap();
                        let t = i64::from_str_radix(t, 10).unwrap();
                        get_logger().log(format_args!("Launch T is {}", t));
                        self.launch_t.replace(t);
                        self.inform_new_t(t).await;
                    } else if line.starts_with(b"curt") {
                        let t = line.split(|t| *t == ',' as u8);
                        let t = core::str::from_utf8(t.skip(1).next().unwrap()).unwrap();
                        let t = i64::from_str_radix(t, 10).unwrap();
                        get_logger().log(format_args!("Current time is {}", t));
                        cortex_m::interrupt::free(|cs| {
                            let mut rtc = unsafe { crate::RTC.borrow(cs) }.borrow_mut();
                            let offset_date_time = OffsetDateTime::from_unix_timestamp(t).unwrap();
                            let date_time = PrimitiveDateTime::new(
                                offset_date_time.date(),
                                offset_date_time.time(),
                            );
                            rtc.as_mut().unwrap().set_datetime(&date_time).unwrap();
                        });
                    } else {
                        hprintln!("Unknown message");
                    }
                }
            }
            bytes = get_serial().read(&mut buffer).await.unwrap();
        }
    }

    async fn inform_new_t(&self, t: i64) {
        for node in NODES.keys() {
            if NODES[node] == NODE_TYPE {
                continue;
            }

            let message = radio::Message::SetLaunchT(t);
            let tx_req = radio::APIFrame::TransmitRequest {
                frame_id: radio::next_id(),
                destination_address: *node,
                broadcast_radius: 3,
                transmit_options: 0,
                data: message.data(),
            };
            let mut buf = [0u8; 512];
            loop {
                tx_req.tx(&mut buf).await;
                match tx_req.await_response().await {
                    APIFrame::ExtendedTransmitStatus { status, .. } => {
                        if status == 0 {
                            break;
                        }
                    }
                    _ => unreachable!(),
                }
            }
        }
        get_serial()
            .write_fmt(format_args!("newt,{}\n", t))
            .unwrap();
    }

    async fn discover_network(self: &Self) {
        self.radio_discover().await;
        self.radio_ping_all().await;
    }

    async fn handle_gps_packets(self: &Self) {
        loop {
            let parse_result = gps::next_sentence().await;

            match parse_result {
                nmea0183::ParseResult::RMC(Some(rmc)) => {
                    get_logger().log(format_args!("RMC: {:?}", rmc));
                    //hprintln!("RMC: {:?}", rmc);

                    if !GPS_HAS_FIX.load(Ordering::SeqCst) {
                        let time = rmc.datetime.time;
                        let date = rmc.datetime.date;
                        cortex_m::interrupt::free(|cs| {
                            let mut rtc_ref = unsafe { crate::RTC.borrow(cs) }.borrow_mut();
                            let rtc = rtc_ref.as_mut().unwrap();

                            let date_time = PrimitiveDateTime::new(
                                Date::from_calendar_date(
                                    date.year as i32,
                                    Month::try_from(date.month).unwrap(),
                                    date.day,
                                )
                                .unwrap(),
                                Time::from_hms_milli(
                                    time.hours,
                                    time.minutes,
                                    time.seconds as u8,
                                    ((time.seconds * 1000.0) as u16) % 1000,
                                )
                                .unwrap(),
                            );

                            rtc.set_datetime(&date_time).unwrap();
                            GPS_HAS_FIX.store(true, Ordering::SeqCst);
                        });
                    }
                }
                nmea0183::ParseResult::GGA(Some(gga)) => {
                    get_logger().log(format_args!("GGA: {:?}", gga));
                    //hprintln!("GGA: {:?}", gga);
                    let msg = radio::Message::Gps {
                        lat: gga.latitude.as_f64(),
                        lon: gga.longitude.as_f64(),
                        alt: gga.altitude.meters as f64,
                    };
                    let mut buf = [0u8; 256];
                    let ground_stations = Self::get_ground_stations();
                    for gs in ground_stations {
                        let tx = radio::APIFrame::TransmitRequest {
                            frame_id: radio::next_id(),
                            destination_address: gs,
                            broadcast_radius: 0,
                            transmit_options: 0,
                            data: msg.data(),
                        };
                        tx.tx(&mut buf).await;
                        tx.await_response().await;
                    }
                }
                _ => {} // Not a sentence we care about
            }
            YieldFuture::new().await;
        }
    }

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
            get_logger().log_str("Sending discover command");

            let response = match api_frame.await_response_timeout(15).await {
                Some(response) => response,
                None => {
                    continue;
                }
            };
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
                                get_logger().log(format_args!(
                                    "info: Peer {:x} found. Identifier: {}",
                                    serial, identifier
                                ));
                                peers_found += 1;
                                if peers_found == PEER_LEN {
                                    break;
                                }
                            }
                            _ => unreachable!(),
                        }
                    } else {
                        get_logger().log(format_args!(
                            "Discovery failed status {}. Retrying.",
                            success
                        ));
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
            let mut pong_recieved = false;
            while !pong_recieved {
                hprintln!("Pinging {:x}", peer);
                get_logger().log(format_args!("info: Pinging {:x}", peer));

                let id = radio::next_id();
                let mut buf = radio::Message::Ping(id).data();
                let api_frame = radio::APIFrame::TransmitRequest {
                    frame_id: id,
                    destination_address: peer,
                    broadcast_radius: 0,
                    transmit_options: 0,
                    data: buf,
                };
                let mut buf2 = [0u8; 512];
                api_frame.tx(&mut buf2).await;

                let status = api_frame.await_response().await;
                match status {
                    APIFrame::ExtendedTransmitStatus { status, .. } => {
                        if status == 0 {
                            radio::claim_frame(|frame| match frame {
                                radio::APIFrame::ReceivePacket {
                                    source_address,
                                    message,
                                    ..
                                } => match message {
                                    Some(radio::Message::Pong(x)) => {
                                        if *source_address == peer && *x == id {
                                            hprintln!("Pong from {:x}", source_address);
                                            get_logger().log(format_args!(
                                                "info: Pong from {:x}",
                                                source_address
                                            ));
                                            pong_recieved = true;
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
                        } else {
                            // loop and resend request.
                        }
                    }
                    _ => unreachable!(),
                }
            }
        }

        hprintln!("All pings ponged");
        get_logger().log(format_args!("info: All pings ponged"));
    }

    async fn report_sensor_data<I2C, TIM>(
        &self,
        mut bmp: bmp388::BMP388<I2C>,
        mut timer: CounterMs<TIM>,
    ) where
        TIM: timer::Instance,
        I2C: embedded_hal::blocking::i2c::WriteRead,
        I2C::Error: core::fmt::Debug,
    {
        timer.start(100.millis()).unwrap();
        loop {
            let values = bmp.sensor_values().unwrap();

            let alt = (1.0 / 0.0065)
                * (pow( 101788.0/(values.pressure), 1.0 / 5.257) - 1.0)
                * (values.temperature + 273.15);

            get_logger().log(format_args!(
                "PTA: {{ pressure: {:.2}, temp: {:.2}, alt: {:.2} }}",
                values.pressure, values.temperature, alt
            ));

            cortex_m::interrupt::free(|cs| {
                let mut alt_readings = self.alt_readings.borrow(cs).borrow_mut();
                if alt_readings.len() == alt_readings.capacity() {
                    alt_readings.pop_front();
                }
                alt_readings.push_back(alt).unwrap();
                //hprintln!("Alt: {:.2}", alt);
            });

            let ground_stations = NODES
                .keys()
                .filter(|x| matches!(NODES[x], NodeType::GroundStation(_)));

            let data = radio::Message::PressureTemp {
                pressure: values.pressure,
                temp: values.temperature,
            }
            .data();

            if false && self.should_send_sensor_data() {
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
            }
            NbFuture::new(|| timer.wait()).await.unwrap();
        }
    }

    fn is_ground_station() -> bool {
        matches!(NODE_TYPE, NodeType::GroundStation(_))
    }

    async fn send_ready_for_ascent(&self) {
        let mut buf = [0u8; 512];
        for gs in Self::get_ground_stations() {
            loop {
                let msg = radio::Message::ReadyForAscent;
                let tx_req = radio::APIFrame::TransmitRequest {
                    frame_id: radio::next_id(),
                    destination_address: gs,
                    broadcast_radius: 0,
                    transmit_options: 0,
                    data: msg.data(),
                };
                tx_req.tx(&mut buf).await;
                let resp = tx_req.await_response().await;
                match resp {
                    APIFrame::ExtendedTransmitStatus { status: 0, .. } => break,
                    _ => {
                        get_logger().log_str("warn: failed to send ReadyForAscent");
                    }
                }
            }
        }
        get_logger().log_str("info: ReadyForAscent sent");
    }
}

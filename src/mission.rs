use core::{
    cell::{Cell, RefCell},
    fmt::{self, Write},
    sync::atomic::{AtomicBool, Ordering},
};

use crate::{
    futures::{NbFuture, YieldFuture},
    get_timestamp, gps,
    radio::{self, APIFrame, LocalCommandResponseData, READINGS_PER_PRTM_PACKET},
    rickroll::{self, rickroll_everyone},
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
    gpio::ExtiPin,
    interrupt,
    prelude::_fugit_DurationExtU32,
    timer::{self, CounterMs},
};
use time::{macros::date, Date, Month, OffsetDateTime, PrimitiveDateTime, Time};

static FORCE_ADVANCE_STAGE: AtomicBool = AtomicBool::new(false);

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

pub const NODE_TYPE: NodeType = NodeType::CanSat(Designator::A);

const NODES: phf::Map<u64, NodeType> = phf_map! {
    //0x0013A20041EFD4CCu64 => NodeType::CanSat(Designator::A),
    0x0013A20041EFD4D5u64 => NodeType::GroundStation(Designator::B),
    0x0013A20041EFD44Au64 => NodeType::CanSat(Designator::A),
    0x0013A20041EFD4CCu64 => NodeType::GroundStation(Designator::A),
    0x0013A20041EFD4D0u64 => NodeType::Rocket
};

const PEER_LEN: usize = 1;
static GPS_HAS_FIX: AtomicBool = AtomicBool::new(false);

const ASCENT_TIMEOUT: i64 = 20;
const DESCENT_TIMEOUT: i64 = 600;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Stage {
    PreLaunch = 0,
    Ascent = 1,
    Descent = 2,
    Recovery = 3,
}

pub struct MissionState {
    launch_t: Cell<i64>,
    stage: Cell<Stage>,
    alt_readings: Mutex<RefCell<Deque<f32, READINGS_PER_PRTM_PACKET>>>,
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
            let frame = radio::try_claim_frame(|frame| {
                let interest = match frame {
                    radio::APIFrame::ReceivePacket {
                        source_address,
                        message: Some(message),
                        ..
                    } => match message {
                        radio::Message::Ping(_) => true,
                        radio::Message::Gps { .. } => true,
                        radio::Message::PressureTemp { .. } => true,
                        radio::Message::SetLaunchT(_) => true,
                        radio::Message::StageChange(_) => false,
                        _ => false,
                    },
                    _ => false,
                };
                interest
            });
            if let Some(radio::APIFrame::ReceivePacket {
                source_address,
                message: Some(message),
                ..
            }) = frame
            {
                match message {
                    radio::Message::Ping(id) => {
                        get_logger().log(format_args!("Received ping from {}", source_address));
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
                    radio::Message::Gps {
                        timestamp,
                        lat,
                        long,
                        alt,
                    } => {
                        write!(
                            get_serial(),
                            "gpsd|{:016x}|{}|{:?}|{:?}|{:?}\n",
                            source_address,
                            timestamp,
                            lat,
                            long,
                            alt
                        )
                        .unwrap();
                    }
                    radio::Message::PressureTemp {
                        timestamp,
                        pressure,
                        temp,
                    } => {
                        write!(
                            get_serial(),
                            "prtp|{:016x}|{}|{:?}|{:?}\n",
                            timestamp,
                            source_address,
                            pressure,
                            temp
                        )
                        .unwrap();
                        let frame = APIFrame::LocalCommandRequest {
                            frame_id: radio::next_id(),
                            command: *b"DB",
                            data: [0u8; 256],
                            data_length: 0,
                        };
                        let mut buf = [0u8; 512];
                        frame.tx(&mut buf).await;
                        let resp = frame.await_response_timeout(3).await;
                        if let Some(APIFrame::LocalCommandResponse {
                            data: LocalCommandResponseData::RSSI { rssi },
                            ..
                        }) = resp
                        {
                            write!(get_serial(), "rssi|{}\n", rssi).unwrap();
                        }
                    }
                    radio::Message::SetLaunchT(t) => {
                        self.launch_t.replace(t);
                        if Self::is_ground_station() {
                            write!(get_serial(), "newt|{}\n", t).unwrap();
                        } else {
                            get_logger().log(format_args!("New launch time {}z", t));
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

        get_logger().log_str("info: Launch!");
        self.stage.replace(Stage::Ascent);
        if !Self::is_ground_station() {
            self.announce_stage_change().await;
        } else {
            self.wait_for_stage_change_messages(Stage::Ascent).await;
        }

        // ASCENT STAGE
        self.wait_for_ascent_completion().await;
        self.stage.replace(Stage::Descent);
        if !Self::is_ground_station() {
            self.announce_stage_change().await;
        } else {
            self.wait_for_stage_change_messages(Stage::Descent).await;
        }
        get_logger().log_str("info: Descent!");

        // DESCENT STAGE
        self.wait_for_descent_completion().await;
        self.stage.replace(Stage::Recovery);
        if !Self::is_ground_station() {
            self.announce_stage_change().await;
        } else {
            self.wait_for_stage_change_messages(Stage::Recovery).await;
        }
        get_logger().log_str("info: Recovery.");

        if Self::is_ground_station() {
            loop {
                YieldFuture::new().await;
            }
        } else {
            rickroll_everyone().await;
        }
    }

    async fn wait_for_stage_change_messages(&self, stage: Stage) {
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
                    radio::Message::StageChange(new_stage) => *new_stage == stage,
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
                    radio::Message::StageChange(stage) => {
                        get_logger().log(format_args!(
                            "{:x} entering stage {:?}",
                            source_address, stage
                        ));
                        write!(get_serial(), "stgc|{:?}\n", stage).unwrap();
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

    async fn announce_stage_change(&self) {
        for gs in Self::get_ground_stations() {
            loop {
                let message = radio::Message::StageChange(self.stage.get());
                let tx_req = radio::APIFrame::TransmitRequest {
                    frame_id: radio::next_id(),
                    destination_address: gs,
                    broadcast_radius: 0,
                    transmit_options: 0,
                    data: message.data(),
                };
                let mut buf = [0; 256];
                tx_req.tx(&mut buf).await;
                match tx_req.await_response_timeout(5).await {
                    Some(APIFrame::ExtendedTransmitStatus { status, .. }) => {
                        if status == 0 {
                            break;
                        }
                    }
                    _ => {}
                };

                get_logger().log(format_args!(
                    "warn: failed to send stage change to {:x}. Retrying.",
                    gs
                ));
            }
        }
    }

    async fn wait_for_ascent_completion(&self) {
        if Self::is_ground_station() {
        } else {
            loop {
                let mut readings = [0.0; READINGS_PER_PRTM_PACKET];
                cortex_m::interrupt::free(|cs| {
                    let alt_readings = self.alt_readings.borrow(cs).borrow();
                    let mut i = 0;
                    for item in alt_readings.iter() {
                        readings[i] = *item;
                        i += 1;
                    }
                });
                let mut diffs = [0.0; READINGS_PER_PRTM_PACKET - 1];
                for i in 0..READINGS_PER_PRTM_PACKET - 1 {
                    diffs[i] = readings[i + 1] - readings[i];
                }
                let average_diff =
                    diffs.iter().sum::<f32>() / (READINGS_PER_PRTM_PACKET - 1) as f32;
                if average_diff < -0.5 {
                    break;
                }

                if FORCE_ADVANCE_STAGE.load(Ordering::SeqCst) {
                    FORCE_ADVANCE_STAGE.store(false, Ordering::SeqCst);
                    break;
                }

                if get_timestamp() - self.launch_t.get() > ASCENT_TIMEOUT {
                    get_logger().log_str("warn: ascent timeout");
                    break;
                }

                YieldFuture::new().await;
            }
        }
    }

    async fn wait_for_descent_completion(&self) {
        loop {
            let mut readings = [0.0; READINGS_PER_PRTM_PACKET];
            cortex_m::interrupt::free(|cs| {
                let alt_readings = self.alt_readings.borrow(cs).borrow();
                let mut i = 0;
                for item in alt_readings.iter() {
                    readings[i] = *item;
                    i += 1;
                }
            });
            let mut diffs = [0.0; READINGS_PER_PRTM_PACKET - 1];
            for i in 0..READINGS_PER_PRTM_PACKET - 1 {
                diffs[i] = readings[i + 1] - readings[i];
            }
            let average_diff = diffs.iter().sum::<f32>() / (READINGS_PER_PRTM_PACKET - 1) as f32;
            if average_diff < -0.03 {
                break;
            }

            if FORCE_ADVANCE_STAGE.load(Ordering::SeqCst) {
                FORCE_ADVANCE_STAGE.store(false, Ordering::SeqCst);
                break;
            }

            if get_timestamp() - self.launch_t.get() > DESCENT_TIMEOUT {
                get_logger().log_str("warn: ascent timeout");
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
            if FORCE_ADVANCE_STAGE.load(Ordering::SeqCst) {
                FORCE_ADVANCE_STAGE.store(false, Ordering::SeqCst);
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
        //gps::tx(b"$PMTK251,115200*1F\r\n").await;
        //gps::change_baudrate(115200).await;
        for _ in 0..1 {
            gps::tx(b"$PMTK220,500*2F\r\n").await;
        }
        while !GPS_HAS_FIX.load(Ordering::SeqCst) {
            YieldFuture::new().await;
        }
    }

    fn address(&self) -> u64 {
        return *NODES
            .keys()
            .filter(|key| NODES[key] == NODE_TYPE)
            .next()
            .unwrap();
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
                        let t = line.split(|t| *t == '|' as u8);
                        let t = core::str::from_utf8(t.skip(1).next().unwrap()).unwrap();
                        let t = i64::from_str_radix(t, 10).unwrap();
                        get_logger().log(format_args!("Launch T is {}", t));
                        self.launch_t.replace(t);
                        self.inform_new_t(t).await;
                    } else if line.starts_with(b"curt") {
                        let t = line.split(|t| *t == '|' as u8);
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
                    } else if line.starts_with(b"pgri") {
                        let address = line.split(|t| *t == '|' as u8);
                        let address =
                            core::str::from_utf8(address.skip(1).next().unwrap()).unwrap();
                        let address = u64::from_str_radix(address, 16).unwrap();
                        get_logger().log(format_args!("Pinging {:x} with route info", address));

                        let mut pong_recieved = false;
                        while !pong_recieved {
                            let id = radio::next_id();
                            let mut buf = radio::Message::Ping(id).data();
                            let api_frame = radio::APIFrame::TransmitRequest {
                                frame_id: id,
                                destination_address: address,
                                broadcast_radius: 0,
                                transmit_options: 0xC8, // Request route info
                                data: buf,
                            };
                            let mut buf2 = [0u8; 512];
                            api_frame.tx(&mut buf2).await;

                            radio::claim_frame_timeout(
                                |frame| match frame {
                                    radio::APIFrame::ReceivePacket {
                                        source_address,
                                        message,
                                        payload,
                                        ..
                                    } => match message {
                                        Some(radio::Message::Pong(x)) => {
                                            if *source_address == address && *x == id {
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
                                },
                                10,
                            )
                            .await;
                            api_frame.await_response().await; // discard tx status
                        }

                        let mut route = Vec::<u64, 8>::new();
                        let mut current = self.address();
                        route.push(current);

                        loop {
                            radio::claim_frame(|frame| match frame {
                                radio::APIFrame::RouteInfo {
                                    responder,
                                    receiver,
                                    ..
                                } => {
                                    if *responder == current {
                                        route.push(*receiver);
                                        current = *receiver;
                                        get_logger().log(format_args!("Receiver: {:x}", receiver));
                                        true
                                    } else {
                                        false
                                    }
                                }
                                _ => false,
                            })
                            .await;

                            if current == address {
                                break;
                            }
                        }

                        write!(get_serial(), "rtin|{:016x?}\n", &route[..]);
                    } else {
                        hprintln!("Unknown message {:?}", line);
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
        // self.radio_discover().await;
        self.radio_ping_all().await;
    }

    async fn handle_gps_packets(self: &Self) {
        let mut lat_readings = [0.0; 8];
        let mut long_readings = [0.0; 8];
        let mut alt_readings = [0.0; 8];
        let mut index = 0;
        loop {
            let parse_result = gps::next_sentence().await;
            match parse_result {
                nmea0183::ParseResult::RMC(Some(rmc)) => {
                    get_logger().log(format_args!(
                        "RMC: {},{},{},{},{:?}",
                        rmc.latitude.as_f64(),
                        rmc.longitude.as_f64(),
                        rmc.speed.as_mps(),
                        rmc.course.map_or(0.0, |x| x.degrees),
                        rmc.mode
                    ));
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
                    get_logger().log(format_args!(
                        "GGA: {},{},{},{},{:?}",
                        gga.latitude.as_f64(),
                        gga.longitude.as_f64(),
                        gga.altitude.meters as f64,
                        gga.hdop,
                        gga.gps_quality,
                    ));
                    //hprintln!("GGA: {:?}", gga);
                    lat_readings[index] = gga.latitude.as_f64();
                    long_readings[index] = gga.longitude.as_f64();
                    alt_readings[index] = gga.altitude.meters as f64;
                    index += 1;
                    if index == 8 {
                        index = 0;
                        if self.should_send_sensor_data() {
                            let msg = radio::Message::Gps {
                                timestamp: get_timestamp(),
                                lat: lat_readings,
                                long: long_readings,
                                alt: alt_readings,
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

                radio::claim_frame_timeout(
                    |frame| match frame {
                        radio::APIFrame::ReceivePacket {
                            source_address,
                            message,
                            payload,
                            ..
                        } => match message {
                            Some(radio::Message::Pong(x)) => {
                                if *source_address == peer && *x == id {
                                    hprintln!("Pong from {:x}", source_address);
                                    get_logger()
                                        .log(format_args!("info: Pong from {:x}", source_address));
                                    pong_recieved = true;
                                    true
                                } else {
                                    false
                                }
                            }
                            _ => false,
                        },

                        _ => false,
                    },
                    10,
                )
                .await;
                api_frame.await_response().await; // discard tx status
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
        let mut pressure_readings = [0.0; READINGS_PER_PRTM_PACKET];
        let mut temperature_readings = [0.0; READINGS_PER_PRTM_PACKET];
        let mut pt_index = 0;
        loop {
            let values = bmp.sensor_values().unwrap();

            let alt = (1.0 / 0.0065)
                * (pow(101788.0 / (values.pressure), 1.0 / 5.257) - 1.0)
                * (values.temperature + 273.15);

            cortex_m::interrupt::free(|cs| {
                let mut alt_readings = self.alt_readings.borrow(cs).borrow_mut();

                if alt_readings.len() == alt_readings.capacity() {
                    alt_readings.pop_front();
                }
                // hprintln!("alt: {:.2}", alt_readings.iter().sum::<f64>()/alt_readings.len() as f64);
                alt_readings.push_back(alt as f32).unwrap();
            });

            let ground_stations = NODES
                .keys()
                .filter(|x| matches!(NODES[x], NodeType::GroundStation(_)));

            pressure_readings[pt_index] = values.pressure as f32;
            temperature_readings[pt_index] = values.temperature as f32;
            pt_index += 1;
            if pt_index == READINGS_PER_PRTM_PACKET {
                pt_index = 0;
                let data = radio::Message::PressureTemp {
                    timestamp: get_timestamp(),
                    pressure: pressure_readings,
                    temp: temperature_readings,
                }
                .data();

                if self.should_send_sensor_data() {
                    for gs in ground_stations {
                        let frame = radio::APIFrame::TransmitRequest {
                            frame_id: radio::next_id(),
                            destination_address: *gs,
                            broadcast_radius: 0,
                            transmit_options: 0,
                            data: data.clone(),
                        };

                        let mut buf = [0u8; 512];
                        frame.tx(&mut buf).await;
                        let resp = frame.await_response().await; // for now discard response
                        hprintln!("info: Sent sensor data to {:x}. {:?}", *gs, resp);
                    }
                }
                get_logger().log(format_args!(
                    "PTA: {{ pressure: {:?}, temp: {:?} }}",
                    pressure_readings, temperature_readings
                ));
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

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs|
        // SAFETY: Mutex makes access of static mutable variable safe
        {
            let mut btn_ref = unsafe { crate::USER_BUTTON.borrow(cs) }.borrow_mut();
            let btn = btn_ref.as_mut().unwrap();
            btn.clear_interrupt_pending_bit();
            FORCE_ADVANCE_STAGE.store(true, Ordering::SeqCst);
    });
}

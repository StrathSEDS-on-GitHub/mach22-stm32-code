use core::{
    borrow::Borrow,
    cell::{BorrowMutError, RefCell},
    cmp::min,
    sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering},
};

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use hal::{
    dma::{Stream7, StreamsTuple},
    gpio,
};
use heapless::{String, Vec};
use stm32f4xx_hal::{
    dma::{
        self,
        traits::{Stream, StreamISR},
        Stream5, Transfer,
    },
    interrupt, pac,
    serial::{Rx, Tx},
};
use time::macros::offset;

use crate::{futures::YieldFuture, get_timestamp, mission, sdcard::get_logger};
use stm32f4xx_hal as hal;

pub struct Lexer<'a> {
    idx: usize,
    data: &'a [u8],
}

impl<'a> Lexer<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { idx: 0, data }
    }

    pub fn remaining_bytes(&self) -> usize {
        self.data.len() - self.idx
    }

    pub fn next_u8(self: &mut Self) -> Option<u8> {
        self.idx += 1;
        self.data.get(self.idx - 1).copied()
    }

    pub fn next_u16(self: &mut Self) -> Option<u16> {
        let it = u16::from_be_bytes(self.data.get(self.idx..self.idx + 2)?.try_into().ok()?);
        self.idx += 2;
        Some(it)
    }

    pub fn next_u64(self: &mut Self) -> Option<u64> {
        let it = u64::from_be_bytes(self.data.get(self.idx..self.idx + 8)?.try_into().ok()?);
        self.idx += 8;
        Some(it)
    }

    pub fn next_f32(self: &mut Self) -> Option<f32> {
        let it = f32::from_be_bytes(self.data.get(self.idx..self.idx + 4)?.try_into().ok()?);
        self.idx += 4;
        Some(it)
    }

    pub fn next_f64(self: &mut Self) -> Option<f64> {
        let it = f64::from_be_bytes(self.data.get(self.idx..self.idx + 8)?.try_into().ok()?);
        self.idx += 8;
        Some(it)
    }

    pub fn next_i64(self: &mut Self) -> Option<i64> {
        let it = i64::from_be_bytes(self.data.get(self.idx..self.idx + 8)?.try_into().ok()?);
        self.idx += 8;
        Some(it)
    }

    pub fn consume(self: &mut Self, x: u8) -> Option<u8> {
        if *self.data.get(self.idx)? == x {
            self.idx += 1;
            Some(x)
        } else {
            None
        }
    }

    pub fn next_str(self: &mut Self, len: usize) -> Option<&'a str> {
        let it =
            unsafe { core::str::from_utf8_unchecked(self.data.get(self.idx..self.idx + len)?) };
        self.idx += len;
        Some(it)
    }

    pub fn next_bytes(self: &mut Self, len: usize) -> Option<&'a [u8]> {
        let it = self.data.get(self.idx..self.idx + len)?;
        self.idx += len;
        Some(it)
    }

    pub fn next_null_terminated_str(self: &mut Self) -> Option<&'a str> {
        let mut idx = self.idx;
        while *self.data.get(idx)? != 0 {
            idx += 1;
        }
        let it = unsafe {
            core::str::from_utf8_unchecked(self.data.get(self.idx..idx)?.try_into().ok()?)
        };
        self.idx = idx + 1;
        Some(it)
    }

    pub fn at_end(self: &Self) -> bool {
        self.idx == self.data.len()
    }

    pub fn skip_till(&mut self, arg: u8) -> Option<u8> {
        while self.next_u8()? != arg {}
        Some(arg)
    }

    fn pos(&self) -> usize {
        self.idx
    }

    pub fn set_pos(&mut self, pos: usize) {
        self.idx = pos;
    }
}

static mut TX_TRANSFER: Mutex<
    RefCell<
        Option<
            Transfer<
                Stream7<pac::DMA2>,
                4,
                Tx<pac::USART1>,
                dma::MemoryToPeripheral,
                &'static mut [u8],
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

static mut RX_TRANSFER: Mutex<
    RefCell<
        Option<
            Transfer<
                Stream5<pac::DMA2>,
                4,
                Rx<pac::USART1>,
                dma::PeripheralToMemory,
                &'static mut [u8],
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

static mut RX_BUFFER: Mutex<RefCell<Option<&'static mut [u8; BUFFER_SIZE]>>> =
    Mutex::new(RefCell::new(None));

static TX_COMPLETE: AtomicBool = AtomicBool::new(false);
static RX_COMPLETE: AtomicBool = AtomicBool::new(false);
static RX_BYTES_READ: AtomicUsize = AtomicUsize::new(0);

const BUFFER_SIZE: usize = 512;

const MAX_PAYLOAD_SIZE: usize = 0x100;

pub struct Parser<'a> {
    lexer: RefCell<Lexer<'a>>,
    buf: &'a [u8],
}

impl<'a> Parser<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Self {
            lexer: RefCell::new(Lexer::new(buf)),
            buf,
        }
    }

    fn pos(&self) -> usize {
        self.lexer.borrow().pos()
    }
}

impl<'a> Iterator for &'a Parser<'_> {
    type Item = Option<APIFrame>;
    fn next(self: &mut Self) -> Option<Self::Item> {
        let mut lexer = self.lexer.borrow_mut();
        let buf = self.buf;

        if buf.len() == 0 || lexer.at_end() {
            return None;
        }

        match lexer.consume(0x7E) {
            Some(_) => {}
            None => {
                hprintln!("warn: incorrect start byte. dropping some unknown bytes");
                lexer.skip_till(0x7E);
            }
        }
        let len = lexer.next_u16()?;

        if lexer.remaining_bytes() < len as usize {
            hprintln!(
                "warn: incorrect length {}, {}",
                len,
                lexer.remaining_bytes()
            );
            // Rewind to the start of the frame
            let pos = lexer.pos();
            lexer.set_pos(pos - 3);
            // The full packet is not yet available
            return None;
        }

        let start = lexer.pos();

        let result = match lexer.next_u8()? {
            0x08 => unimplemented!(),
            0x88 => {
                let frame_id = lexer.next_u8()?;
                let command_name = lexer.next_str(2)?;
                let command_success = lexer.next_u8()?;
                if command_success != 0 {
                    Some(Some(APIFrame::LocalCommandResponse {
                        frame_id,
                        command: command_name.as_bytes().try_into().unwrap(),
                        success: command_success,
                        data: LocalCommandResponseData::NoData,
                    }));
                }
                match command_name {
                    "ND" => {
                        lexer.next_u16(); // unused.

                        let serial = lexer.next_u64()?;
                        let signal_strength = lexer.next_u8()?;
                        let identifier_str = lexer.next_null_terminated_str()?;
                        let mut identifier = String::from(identifier_str);

                        lexer.next_u16()?; // unused

                        let device_type = lexer.next_u8()?;
                        let status = lexer.next_u8()?;
                        let profile_id = lexer.next_u16()?;
                        let manufacturer = lexer.next_u16()?;

                        Some(Some(APIFrame::LocalCommandResponse {
                            frame_id,
                            command: command_name.as_bytes().try_into().unwrap(),
                            success: command_success,
                            data: LocalCommandResponseData::NetworkDiscoverResponse {
                                serial,
                                signal_strength,
                                identifier,
                                device_type,
                                status,
                                profile_id,
                                manufacturer,
                            },
                        }))
                    }
                    _ => todo!("NYI: {}", command_name),
                }
            }
            0x8b => {
                let frame_id = lexer.next_u8()?;
                lexer.next_u16(); // reserved
                let retries = lexer.next_u8()?;
                let status = lexer.next_u8()?;
                let discovery_status = lexer.next_u8()?;
                Some(Some(APIFrame::ExtendedTransmitStatus {
                    frame_id,
                    retries,
                    status,
                    discovery_status,
                }))
            }
            0x90 => {
                let source_address = lexer.next_u64()?;
                lexer.next_u16()?; // reserved
                let recv_options = lexer.next_u8()?;
                let data_len = len as usize - 12; // 12 = frame_type + src_addr + recv_options + reserved
                let data_recv = lexer.next_bytes(data_len)?;
                let mut payload = Vec::new();
                payload.extend_from_slice(data_recv).unwrap();

                let message = Message::parse(&payload);
                Some(Some(APIFrame::ReceivePacket {
                    source_address,
                    recv_options,
                    payload,
                    message,
                }))
            }
            _ => {
                todo!("NYI: {:x?}", &buf);
            }
        };

        lexer.next_u8()?; // checksum byte

        let checksum = buf[start..1 + start + len as usize]
            .iter()
            .fold(0, |acc: u8, x| acc.wrapping_add(*x));
        if checksum != 0xFF {
            hprintln!(
                "warn: checksum mismatch {} {:02x?} {} {}",
                checksum,
                &buf[start..1 + start + len as usize],
                start,
                len
            );
            return None;
        }

        result
    }
}

#[derive(Debug)]
pub enum LocalCommandResponseData {
    NetworkDiscoverResponse {
        serial: u64,
        signal_strength: u8,
        identifier: String<20>,
        device_type: u8,
        status: u8,
        profile_id: u16,
        manufacturer: u16,
    },
    NoData,
}

#[derive(Debug)]
pub enum APIFrame {
    LocalCommandRequest {
        frame_id: u8,
        command: [u8; 2],
        data: [u8; MAX_PAYLOAD_SIZE],
        data_length: usize,
    },
    LocalCommandResponse {
        frame_id: u8,
        command: [u8; 2],
        success: u8,
        data: LocalCommandResponseData,
    },
    TransmitRequest {
        frame_id: u8,
        destination_address: u64,
        broadcast_radius: u8,
        transmit_options: u8,
        data: Vec<u8, MAX_PAYLOAD_SIZE>,
    },
    ReceivePacket {
        source_address: u64,
        recv_options: u8,
        payload: Vec<u8, MAX_PAYLOAD_SIZE>,
        message: Option<Message>,
    },
    ExtendedTransmitStatus {
        frame_id: u8,
        retries: u8,
        status: u8,
        discovery_status: u8,
    },
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum Message {
    Ping(u8),
    Pong(u8),
    Gps { lat: f64, lon: f64, alt: f64 },
    PressureTemp { pressure: f64, temp: f64 },
    ReadyForAscent,
    SetLaunchT(i64),
    StageChange(mission::Stage),
}

impl Message {
    pub fn parse(payload: &[u8]) -> Option<Self> {
        let mut lexer = Lexer::new(payload);
        let message_type = lexer.next_str(4)?;
        match message_type {
            "PING" => {
                let ping_id = lexer.next_u8()?;
                Some(Message::Ping(ping_id))
            }
            "PONG" => {
                let ping_id = lexer.next_u8()?;
                Some(Message::Pong(ping_id))
            }
            "GPSD" => {
                let lat = lexer.next_f64()?;
                let lon = lexer.next_f64()?;
                let alt = lexer.next_f64()?;
                Some(Message::Gps { lat, lon, alt })
            }
            "PRTM" => {
                let pressure = lexer.next_f64()?;
                let temp = lexer.next_f64()?;
                Some(Message::PressureTemp { pressure, temp })
            }
            "RDYA" => Some(Message::ReadyForAscent),
            "STGC" => {
                let stage = match lexer.next_u8()? {
                    0 => mission::Stage::PreLaunch,
                    1 => mission::Stage::Ascent,
                    2 => mission::Stage::Descent,
                    3 => mission::Stage::Recovery,
                    _ => {
                        return None;
                    }
                };
                Some(Message::StageChange(stage))
            }
            "SETT" => {
                let t = lexer.next_i64()?;
                Some(Message::SetLaunchT(t))
            }
            _ => None,
        }
    }

    pub fn data(&self) -> Vec<u8, MAX_PAYLOAD_SIZE> {
        // get_logger().log(format_args!("{:?}", self));
        match self {
            Message::Ping(ping_id) => {
                let mut payload = Vec::new();
                payload.extend_from_slice(b"PING");
                payload.push(*ping_id);
                payload
            }
            Message::Pong(ping_id) => {
                let mut payload = Vec::new();
                payload.extend_from_slice(b"PONG");
                payload.push(*ping_id);
                payload
            }
            Message::Gps { lat, lon, alt } => {
                let mut payload = Vec::new();
                payload.extend_from_slice(b"GPSD");
                payload.extend_from_slice(&lat.to_be_bytes());
                payload.extend_from_slice(&lon.to_be_bytes());
                payload.extend_from_slice(&alt.to_be_bytes());
                payload
            }
            Message::PressureTemp { pressure, temp } => {
                let mut payload = Vec::new();
                payload.extend_from_slice(b"PRTM");
                payload.extend_from_slice(&pressure.to_be_bytes());
                payload.extend_from_slice(&temp.to_be_bytes());
                payload
            }
            Message::ReadyForAscent => {
                let mut payload = Vec::new();
                payload.extend_from_slice(b"RDYA");
                payload
            }
            Message::SetLaunchT(t) => {
                let mut payload = Vec::new();
                payload.extend_from_slice(b"SETT");
                payload.extend_from_slice(&t.to_be_bytes());
                payload
            }
            Message::StageChange(stage) => {
                let mut payload = Vec::new();
                payload.extend_from_slice(b"STGC");
                payload.extend_from_slice(&[*stage as u8]);
                payload
            }
        }
    }
}

static ID_COUNTER: AtomicU8 = AtomicU8::new(1);

pub fn next_id() -> u8 {
    ID_COUNTER
        .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |x| {
            Some(if x == 255 { 1 } else { x + 1 })
        })
        .unwrap()
}

static mut UNHANDLED_FRAMES: Mutex<RefCell<Vec<APIFrame, 32>>> =
    Mutex::new(RefCell::new(Vec::new()));

pub fn try_claim_frame<F>(mut f: F) -> Option<APIFrame>
where
    F: FnMut(&APIFrame) -> bool,
{
    cortex_m::interrupt::free(|cs| {
        let mut frames = unsafe { UNHANDLED_FRAMES.borrow(cs) }.borrow_mut();
        for i in 0..frames.len() {
            if f(&frames[i]) {
                let frame = frames.swap_remove(i);
                return Some(frame);
            }
        }
        None
    })
}

pub async fn claim_frame<F>(mut f: F) -> Option<APIFrame>
where
    F: FnMut(&APIFrame) -> bool,
{
    loop {
        if let Some(frame) = cortex_m::interrupt::free(|cs| {
            let mut frames = unsafe { UNHANDLED_FRAMES.borrow(cs) }.borrow_mut();
            for i in 0..frames.len() {
                if f(&frames[i]) {
                    let frame = frames.swap_remove(i);
                    return Some(frame);
                }
            }
            None
        }) {
            return Some(frame);
        }
        YieldFuture::new().await;
    }
}

impl APIFrame {
    pub fn local_command_request(
        frame_id: u8,
        command: [u8; 2],
        data: [u8; MAX_PAYLOAD_SIZE],
        data_length: usize,
    ) -> Self {
        APIFrame::LocalCommandRequest {
            frame_id,
            command,
            data,
            data_length,
        }
    }

    pub fn serialize(self: &Self, buf: &mut [u8]) -> Option<usize> {
        let mut bytes = 3;
        *buf.get_mut(0)? = 0x7E;

        match self {
            APIFrame::LocalCommandRequest {
                frame_id,
                command,
                data,
                data_length,
            } => {
                bytes += 4;
                *buf.get_mut(bytes - 4)? = 0x08;
                *buf.get_mut(bytes - 3)? = *frame_id;
                *buf.get_mut(bytes - 2)? = command[0];
                *buf.get_mut(bytes - 1)? = command[1];

                if buf.len() - bytes < *data_length {
                    return None;
                }

                buf[6..6 + *data_length].copy_from_slice(&data[..*data_length]);
                bytes += data_length;
            }
            APIFrame::LocalCommandResponse { .. } => unimplemented!(),
            APIFrame::TransmitRequest {
                frame_id,
                destination_address,
                broadcast_radius,
                transmit_options,
                data,
                ..
            } => {
                *buf.get_mut(bytes)? = 0x10;
                *buf.get_mut(bytes + 1)? = *frame_id;
                buf.get_mut(bytes + 2..bytes + 10)?
                    .copy_from_slice(&destination_address.to_be_bytes());
                *buf.get_mut(bytes + 10)? = 0xFF;
                *buf.get_mut(bytes + 11)? = 0xFE;
                *buf.get_mut(bytes + 12)? = *broadcast_radius;
                *buf.get_mut(bytes + 13)? = *transmit_options;
                buf.get_mut(bytes + 14..bytes + 14 + data.len())?
                    .copy_from_slice(&data[..data.len()]);
                bytes += 14 + data.len();
            }
            APIFrame::ReceivePacket { .. } => unimplemented!(),
            APIFrame::ExtendedTransmitStatus { .. } => unimplemented!(),
        }

        *buf.get_mut(1)? = (bytes - 3 >> 8) as u8;
        *buf.get_mut(2)? = (bytes - 3 & 0xFF) as u8;

        let checksum = 0xFF
            - buf[3..bytes]
                .iter()
                .fold(0, |acc: u8, &x| acc.wrapping_add(x));
        *buf.get_mut(bytes)? = checksum;
        bytes += 1;
        // hprintln!("{:x?}", &buf[..bytes]);
        Some(bytes)
    }

    pub async fn tx(self: &Self, buf: &mut [u8]) {
        let bytes = self.serialize(buf).unwrap();
        crate::radio::tx(&buf[..bytes]).await;
    }

    pub async fn await_response(self: &Self) -> APIFrame {
        loop {
            if let Some(frame) = try_claim_frame(|frame| self.frame_id() == frame.frame_id()) {
                return frame;
            }
            YieldFuture::new().await;
        }
    }

    pub async fn await_response_timeout(self: &Self, timeout_secs: i64) -> Option<APIFrame> {
        let start_time = get_timestamp();
        loop {
            if let Some(frame) = try_claim_frame(|frame| self.frame_id() == frame.frame_id()) {
                return Some(frame);
            }
            if get_timestamp() - start_time > timeout_secs {
                return None;
            }
            YieldFuture::new().await;
        }
    }

    pub fn frame_id(self: &Self) -> u8 {
        match self {
            APIFrame::LocalCommandRequest { frame_id, .. } => *frame_id,
            APIFrame::LocalCommandResponse { frame_id, .. } => *frame_id,
            APIFrame::TransmitRequest { frame_id, .. } => *frame_id,
            APIFrame::ReceivePacket { .. } => 0, // not applicable
            APIFrame::ExtendedTransmitStatus { frame_id, .. } => *frame_id,
        }
    }
}

pub fn setup(
    DMA2: pac::DMA2,
    radio: hal::serial::Serial<
        pac::USART1,
        (
            gpio::Pin<'A', 15, gpio::Alternate<7>>,
            gpio::Pin<'A', 10, gpio::Alternate<7>>,
        ),
    >,
) {
    let streams = StreamsTuple::new(DMA2);
    let tx_stream = streams.7;
    let rx_stream = streams.5;
    let tx_buf_radio = cortex_m::singleton!(:[u8; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap();
    let rx_buf1_radio = cortex_m::singleton!(:[u8; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap();
    let rx_buf2_radio = cortex_m::singleton!(:[u8; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap();
    let (tx, mut rx) = radio.split();
    let mut tx_transfer = Transfer::init_memory_to_peripheral(
        tx_stream,
        tx,
        tx_buf_radio as &mut [u8],
        None,
        dma::config::DmaConfig::default()
            .memory_increment(true)
            .fifo_enable(true)
            .transfer_complete_interrupt(true),
    );
    rx.listen_idle();
    let mut rx_transfer = Transfer::init_peripheral_to_memory(
        rx_stream,
        rx,
        rx_buf1_radio as &mut [u8],
        None,
        dma::config::DmaConfig::default()
            .memory_increment(true)
            .fifo_enable(true)
            .fifo_error_interrupt(true)
            .transfer_error_interrupt(true)
            .direct_mode_error_interrupt(true)
            .transfer_complete_interrupt(true),
    );
    rx_transfer.start(|_rx| {});
    tx_transfer.start(|_tx| {});
    cortex_m::interrupt::free(|cs| {
        // SAFETY: Mutex makes access of static mutable variable safe
        unsafe { TX_TRANSFER.borrow(cs) }.replace(Some(tx_transfer));
        unsafe { RX_TRANSFER.borrow(cs) }.replace(Some(rx_transfer));
        unsafe { RX_BUFFER.borrow(cs) }.replace(Some(rx_buf2_radio));
    });
    cortex_m::interrupt::free(|cs| {
        let mut tx_ref = unsafe { TX_TRANSFER.borrow(cs) }.borrow_mut();
    });
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA2_STREAM7);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA2_STREAM5);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART1);
    }
}

/// Interrupt for radio DMA TX,
#[interrupt]
fn DMA2_STREAM7() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = unsafe { TX_TRANSFER.borrow(cs) }.borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        // Its important to clear fifo errors as the transfer is paused until it is cleared
        if Stream7::<pac::DMA2>::get_fifo_error_flag() {
            transfer.clear_fifo_error_interrupt();
        }

        if Stream7::<pac::DMA2>::get_transfer_complete_flag() {
            transfer.clear_transfer_complete_interrupt();
            // FIXME: A less restrictive ordering is probably possible
            TX_COMPLETE.store(true, Ordering::SeqCst);
        }

    });
}

// Call this to poll for new frames.
pub async fn parse_recvd_data() {
    let mut rx_buf = [0u8; BUFFER_SIZE];
    let mut bytes = crate::radio::rx(&mut rx_buf).await;
    let mut parser = Parser::new(&rx_buf[..bytes]);
    loop {
        // hprintln!("Received frame {:x?}", &rx_buf[..bytes]);
        for frame in &parser {
            if let Some(frame) = frame {
                // get_logger().log(format_args!("Received frame {:x?}", frame));
                cortex_m::interrupt::free(|cs| {
                    let r = unsafe { UNHANDLED_FRAMES.borrow(cs) }
                        .borrow_mut()
                        .push(frame);
                    match r {
                        Ok(_) => (),
                        Err(_) => {
                            hprintln!("warn: unhandled frame buffer full. dropping a frame.")
                        }
                    }
                });
            } else {
                hprintln!("warn: invalid frame received");
            }
        }

        // Make space in buffer and try to receive more data.
        let pos = parser.pos();
        for i in pos..bytes {
            rx_buf[i - pos] = rx_buf[i];
        }
        let unparsed_bytes = bytes - pos;
        bytes = crate::radio::rx(&mut rx_buf[unparsed_bytes..]).await + unparsed_bytes;
        parser = Parser::new(&rx_buf[..bytes]);
        YieldFuture::new().await;
    }
}

// Interrupt for radio DMA RX.
#[interrupt]
fn DMA2_STREAM5() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = unsafe { RX_TRANSFER.borrow(cs) }.borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        // Its important to clear fifo errors as the transfer is paused until it is cleared
        if Stream5::<pac::DMA2>::get_fifo_error_flag() {
            transfer.clear_fifo_error_interrupt();
        }
        if Stream5::<pac::DMA2>::get_transfer_complete_flag() {
            transfer.clear_transfer_complete_interrupt();
            let mut rx_buf = unsafe { RX_BUFFER.borrow(cs) }.borrow_mut().take().unwrap();
            rx_buf = transfer
                .next_transfer(rx_buf)
                .unwrap()
                .0
                .try_into()
                .unwrap();
            RX_COMPLETE.store(true, Ordering::SeqCst);
            RX_BYTES_READ.store(rx_buf.len(), Ordering::SeqCst);
            unsafe { RX_BUFFER.borrow(cs) }.borrow_mut().replace(rx_buf);
        }
    });
}

#[interrupt]
fn USART1() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = unsafe { RX_TRANSFER.borrow(cs) }.borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        let bytes = BUFFER_SIZE as u16 - Stream5::<pac::DMA2>::get_number_of_transfers();
        transfer.pause(|rx| {
            rx.clear_idle_interrupt();
        });

        let mut rx_buf = unsafe { RX_BUFFER.borrow(cs) }.borrow_mut().take().unwrap();

        rx_buf = transfer
            .next_transfer(rx_buf)
            .unwrap()
            .0
            .try_into()
            .unwrap();
        RX_COMPLETE.store(true, Ordering::SeqCst);
        RX_BYTES_READ.store(bytes as usize, Ordering::SeqCst);

        unsafe { RX_BUFFER.borrow(cs) }.borrow_mut().replace(rx_buf);
        transfer.start(|_| {});
    });
}

async fn radio_tx_complete() {
    loop {
        if TX_COMPLETE.load(Ordering::SeqCst) {
            break;
        }

        YieldFuture::new().await;
    }
}

async fn radio_rx_complete() {
    loop {
        if RX_COMPLETE.load(Ordering::SeqCst) {
            break;
        }

        YieldFuture::new().await;
    }
}

pub async fn tx(tx_buf: &[u8]) {
    radio_tx_complete().await;
    cortex_m::interrupt::free(|cs| {
        let mut tx_transfer_ref = unsafe { TX_TRANSFER.borrow(cs) }.borrow_mut();
        let tx_transfer = tx_transfer_ref.as_mut().unwrap();
        TX_COMPLETE.store(false, Ordering::SeqCst);
        // SAFETY: not double buffered.
        unsafe {
            tx_transfer
                .next_transfer_with(|buf, _| {
                    buf[..tx_buf.len()].copy_from_slice(&tx_buf);
                    (buf, ())
                })
                .unwrap();
        }
    });
    radio_tx_complete().await;
}

pub async fn rx(rx_buf: &mut [u8]) -> usize {
    radio_rx_complete().await;
    cortex_m::interrupt::free(|cs| {
        // SAFETY: not double buffered.
        let mut buf_ref = unsafe { RX_BUFFER.borrow(cs) }.borrow_mut();
        let buf = buf_ref.as_mut().unwrap();
        let bytes_available = RX_BYTES_READ.load(Ordering::SeqCst) as usize;

        let bytes_copied = min(rx_buf.len(), bytes_available);
        rx_buf[..bytes_copied].copy_from_slice(&buf[..bytes_copied]);
        RX_COMPLETE.store(bytes_available - bytes_copied != 0, Ordering::SeqCst);
        RX_BYTES_READ.store(bytes_available - bytes_copied, Ordering::SeqCst);

        for i in bytes_copied..bytes_available {
            buf[i - bytes_copied] = buf[i]
        }

        bytes_copied
    })
}

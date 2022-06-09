use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use hal::{dma::StreamsTuple, gpio};
use stm32f4xx_hal::{
    dma::{
        self,
        traits::{Stream, StreamISR},
        Stream5, Stream7, Transfer,
    },
    interrupt, pac,
    serial::{Rx, Tx},
};

use crate::futures::YieldFuture;
use stm32f4xx_hal as hal;

pub struct Parser<'a> {
    idx: usize,
    data: &'a [u8],
}

impl<'a> Parser<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { idx: 0, data }
    }
    pub fn next_u8(self: &mut Self) -> Option<u8> {
        self.idx += 1;
        self.data.get(self.idx - 1).copied()
    }

    pub fn next_u16(self: &mut Self) -> Option<u16> {
        let it = u16::from_le_bytes(self.data.get(self.idx..self.idx + 2)?.try_into().ok()?);
        self.idx += 2;
        Some(it)
    }

    pub fn next_u64(self: &mut Self) -> Option<u64> {
        let it = u64::from_le_bytes(self.data.get(self.idx..self.idx + 8)?.try_into().ok()?);
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

    pub fn next_null_terminated_str(self: &mut Self) -> Option<&'a str> {
        let mut idx = self.idx;
        while *self.data.get(idx)? != 0 {
            idx += 1;
        }
        let it = unsafe {
            core::str::from_utf8_unchecked(self.data.get(self.idx..idx + 1)?.try_into().ok()?)
        };
        self.idx = idx + 1;
        Some(it)
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

static mut RX_BUFFER: Mutex<RefCell<Option<&'static mut [u8; RX_BUFFER_SIZE]>>> =
    Mutex::new(RefCell::new(None));

static TX_COMPLETE: AtomicBool = AtomicBool::new(false);
static RX_COMPLETE: AtomicBool = AtomicBool::new(false);
static RX_BYTES_READ: AtomicU32 = AtomicU32::new(0);

const RX_BUFFER_SIZE: usize = 32;

const MAX_COMMAND_DATA_SIZE: usize = 32;

#[derive(Debug)]
pub enum LocalCommandResponseData {
    NetworkDiscoverResponse {
        serial: u64,
        signal_strength: u8,
        identifier: [u8; 20],
        identifier_len: usize,
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
        data: [u8; MAX_COMMAND_DATA_SIZE],
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
        data: [u8; MAX_COMMAND_DATA_SIZE],
        data_length: usize,
    },
}

impl APIFrame {
    pub fn local_command_request(
        frame_id: u8,
        command: [u8; 2],
        data: [u8; MAX_COMMAND_DATA_SIZE],
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
                data_length,
            } => {
                *buf.get_mut(bytes)? = 0x10;
                *buf.get_mut(bytes + 1)? = *frame_id;
                buf.get_mut(bytes + 2..bytes + 10)?
                    .copy_from_slice(&destination_address.to_be_bytes());
                *buf.get_mut(bytes + 10)? = 0xFF;
                *buf.get_mut(bytes + 11)? = 0xFE;
                *buf.get_mut(bytes + 12)? = *broadcast_radius;
                *buf.get_mut(bytes + 13)? = *transmit_options;
                buf.get_mut(bytes + 14..bytes + 14 + *data_length)?
                    .copy_from_slice(&data[..*data_length]);
                bytes += 14 + *data_length;
            }
        }

        *buf.get_mut(1)? = (bytes - 3 >> 8) as u8;
        *buf.get_mut(2)? = (bytes - 3 & 0xFF) as u8;

        let checksum = 0xFF
            - buf[3..bytes]
                .iter()
                .fold(0, |acc: u8, &x| acc.wrapping_add(x));
        *buf.get_mut(bytes)? = checksum;
        bytes += 1;
        hprintln!("{:x?}", &buf[..bytes]);
        Some(bytes)
    }

    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < 3 {
            return None;
        }
        let mut parser = Parser::new(buf);
        parser.consume(0x7E);
        let len = parser.next_u16()?;

        match parser.next_u8()? {
            0x08 => unimplemented!(),
            0x88 => {
                let frame_id = parser.next_u8()?;
                let command_name = parser.next_str(2)?;
                let command_success = parser.next_u8()?;
                if command_success != 0 {
                    return Some(APIFrame::LocalCommandResponse {
                        frame_id,
                        command: command_name.as_bytes().try_into().unwrap(),
                        success: command_success,
                        data: LocalCommandResponseData::NoData,
                    });
                }
                match command_name {
                    "ND" => {
                        parser.next_u16(); // unused.

                        let serial = parser.next_u64()?;
                        let signal_strength = parser.next_u8()?;
                        let identifier_str = parser.next_null_terminated_str()?;
                        let mut identifier = [0u8; 20];
                        identifier[..identifier_str.len()]
                            .copy_from_slice(identifier_str.as_bytes());

                        let device_type = parser.next_u8()?;
                        let status = parser.next_u8()?;
                        let profile_id = parser.next_u16()?;
                        let manufacturer = parser.next_u16()?;

                        Some(APIFrame::LocalCommandResponse {
                            frame_id,
                            command: command_name.as_bytes().try_into().unwrap(),
                            success: command_success,
                            data: LocalCommandResponseData::NetworkDiscoverResponse {
                                serial,
                                signal_strength,
                                identifier,
                                identifier_len: identifier_str.len(),
                                device_type,
                                status,
                                profile_id,
                                manufacturer,
                            },
                        })
                    }
                    _ => todo!("NYI: {}", command_name),
                }
            }
            _ => {
                todo!("NYI: {:x?}", &buf);
            }
        }
    }

    pub async fn tx(self: &Self, buf: &mut [u8]) {
        let bytes = self.serialize(buf).unwrap();
        crate::radio::tx(&buf[..bytes]).await;
    }
}

pub fn setup(
    dma2: pac::DMA2,
    radio: hal::serial::Serial<
        pac::USART1,
        (
            gpio::Pin<'A', 15, gpio::Alternate<7>>,
            gpio::Pin<'A', 10, gpio::Alternate<7>>,
        ),
    >,
) {
    let streams = StreamsTuple::new(dma2);
    let tx_stream = streams.7;
    let rx_stream = streams.5;
    let tx_buf = cortex_m::singleton!(:[u8; 32] = [0; 32]).unwrap();
    let rx_buf1 = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let rx_buf2 = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let (tx, mut rx) = radio.split();
    let mut tx_transfer = Transfer::init_memory_to_peripheral(
        tx_stream,
        tx,
        tx_buf as &mut [u8],
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
        rx_buf1 as &mut [u8],
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
        unsafe { RX_BUFFER.borrow(cs) }.replace(Some(rx_buf2));
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
            RX_BYTES_READ.store(rx_buf.len() as u32, Ordering::SeqCst);
            unsafe { RX_BUFFER.borrow(cs) }.borrow_mut().replace(rx_buf);
        }
    });
}

#[interrupt]
fn USART1() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = unsafe { RX_TRANSFER.borrow(cs) }.borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        let bytes = RX_BUFFER_SIZE as u16 - Stream5::<pac::DMA2>::get_number_of_transfers();
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
        //hprintln!("idle: {:?} {}", rx_buf, bytes);
        RX_COMPLETE.store(true, Ordering::SeqCst);
        RX_BYTES_READ.store(bytes as u32, Ordering::SeqCst);

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
    RX_COMPLETE.store(false, Ordering::SeqCst);
    cortex_m::interrupt::free(|cs| {
        // SAFETY: not double buffered.
        let rx_buf_ref = unsafe { RX_BUFFER.borrow(cs) }.borrow();
        let buf = rx_buf_ref.as_ref().unwrap();
        rx_buf[..buf.len()].copy_from_slice(*buf);
    });
    RX_BYTES_READ.load(Ordering::SeqCst) as usize
}

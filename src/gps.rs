use core::{
    cell::RefCell,
    cmp::min,
    marker::PhantomData,
    pin::Pin,
    sync::atomic::{AtomicBool, AtomicU32, AtomicU8, AtomicUsize, Ordering},
};

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::{hprint, hprintln};
use hal::{
    dma::{Stream6, StreamsTuple},
    gpio::{self, gpioa, Alternate, GpioExt, Input, Output},
    pac::{Peripherals, USART2},
    prelude::_stm32f4xx_hal_time_U32Ext,
    serial::{Serial, SerialExt},
};
use heapless::{Deque, String, Vec};
use nmea0183::ParseResult;
use stm32f4xx_hal::{
    dma::{
        self,
        traits::{Stream, StreamISR},
        Stream5, Transfer,
    },
    interrupt, pac,
    serial::{Rx, Tx},
};

use crate::{
    futures::YieldFuture,
    sdcard::{get_logger, SdLogger},
};
use stm32f4xx_hal as hal;

static mut TX_TRANSFER: Mutex<
    RefCell<
        Option<
            Transfer<
                Stream6<pac::DMA1>,
                4,
                Tx<pac::USART2>,
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
                Stream5<pac::DMA1>,
                4,
                Rx<pac::USART2>,
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
static RX_BYTES_READ: AtomicUsize = AtomicUsize::new(0);

struct FakeUSART2 {
    _marker: PhantomData<*const ()>,
}

struct FakePin<const P: char, const N: u8, MODE = gpio::Input> {
    _mode: PhantomData<MODE>,
}

const RX_BUFFER_SIZE: usize = 512;
pub fn setup(
    dma1: pac::DMA1,
    gps: hal::serial::Serial<
        pac::USART2,
        (
            gpio::Pin<'A', 2, gpio::Alternate<7>>,
            gpio::Pin<'A', 3, gpio::Alternate<7>>,
        ),
    >,
) {
    let streams = StreamsTuple::new(dma1);
    let tx_stream = streams.6;
    let rx_stream = streams.5;
    let tx_buf_gps = cortex_m::singleton!(:[u8; 32] = [0; 32]).unwrap();
    let rx_buf1_gps = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let rx_buf2_gps = cortex_m::singleton!(:[u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE]).unwrap();
    let (tx, mut rx) = gps.split();
    let mut tx_transfer = Transfer::init_memory_to_peripheral(
        tx_stream,
        tx,
        tx_buf_gps as &mut [u8],
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
        rx_buf1_gps as &mut [u8],
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
        unsafe { RX_BUFFER.borrow(cs) }.replace(Some(rx_buf2_gps));

        unsafe { GPS_SENTENCE_BUFFER.borrow(cs) }.replace(Some(Deque::new()))
    });
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM6);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM5);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
    }
}

pub async fn change_baudrate(baudrate: u32) {
    //gps_tx_complete().await;
    cortex_m::interrupt::free(|cs| {
        // SAFETY: Mutex makes access of static mutable variable safe
        let tx_transfer = unsafe { TX_TRANSFER.borrow(cs) }.replace(None).unwrap();
        let rx_transfer = unsafe { RX_TRANSFER.borrow(cs) }.replace(None).unwrap();

        let (rx_stream, rx, rx_buf, _) = rx_transfer.release();
        let (tx_stream, tx, tx_buf, _) = tx_transfer.release();

        let serial = {
            // Seemingly there's no way to simply rejoin the Tx and Rx and then release so we need to get our hands dirty...
            let mut clocks_ref = unsafe { crate::CLOCKS.borrow(cs) }.borrow_mut();
            let clocks = clocks_ref.as_mut().unwrap();

            // FIXME: Holy shit what the fuck is this code
            let usart2: USART2 = unsafe {
                core::mem::transmute(FakeUSART2 {
                    _marker: PhantomData,
                })
            };
            let pa2 = unsafe {
                core::mem::transmute::<FakePin<'A', 2, Input>, gpio::Pin<'A', 2, Input>>(FakePin {
                    _mode: PhantomData::<gpio::Input>,
                })
            };

            let pa3 = unsafe {
                core::mem::transmute::<FakePin<'A', 3, Input>, gpio::Pin<'A', 3, Input>>(FakePin {
                    _mode: PhantomData::<gpio::Input>,
                })
            };
            usart2
                .serial(
                    (pa2.into_alternate(), pa3.into_alternate()),
                    hal::serial::config::Config {
                        baudrate: baudrate.bps(),
                        dma: hal::serial::config::DmaConfig::TxRx,
                        ..Default::default()
                    },
                    clocks,
                )
                .unwrap()
        };
        let (tx, mut rx) = serial.split();
        rx.listen_idle();
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
            rx_buf as &mut [u8],
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

        RX_COMPLETE.store(false, Ordering::SeqCst);
        RX_BYTES_READ.store(0, Ordering::SeqCst);
        // SAFETY: Mutex makes access of static mutable variable safe
        unsafe { TX_TRANSFER.borrow(cs) }.replace(Some(tx_transfer));
        unsafe { RX_TRANSFER.borrow(cs) }.replace(Some(rx_transfer));
    });
}

static mut GPS_SENTENCE_BUFFER: Mutex<RefCell<Option<Deque<ParseResult, 64>>>> =
    Mutex::new(RefCell::new(None));

pub async fn next_sentence() -> ParseResult {
    loop {
        if let Some(x) = cortex_m::interrupt::free(|cs| {
            let next = unsafe { GPS_SENTENCE_BUFFER.borrow(cs) }
                .borrow_mut()
                .as_mut()
                .unwrap()
                .pop_front();

            next
        }) {
            return x;
        }

        YieldFuture::new().await;
    }
}

/// Interrupt for radio DMA TX,
#[interrupt]
fn DMA1_STREAM6() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = unsafe { TX_TRANSFER.borrow(cs) }.borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        // Its important to clear fifo errors as the transfer is paused until it is cleared
        if Stream6::<pac::DMA1>::get_fifo_error_flag() {
            transfer.clear_fifo_error_interrupt();
        }

        if Stream6::<pac::DMA1>::get_transfer_complete_flag() {
            transfer.clear_transfer_complete_interrupt();
            // FIXME: A less restrictive ordering is probably possible
            TX_COMPLETE.store(true, Ordering::SeqCst);
        }
    });
}

// Call this to poll for new frames.
pub async fn parse_recvd_data() {
    let mut rx_buf = [0u8; RX_BUFFER_SIZE];
    let mut bytes = crate::gps::rx(&mut rx_buf).await;
    let mut parser = nmea0183::Parser::new();

    loop {
        // hprintln!("{:?}", core::str::from_utf8(&rx_buf[..bytes]));
        for parse_result in parser.parse_from_bytes(&rx_buf[..bytes]) {
            if let Ok(parse_result) = parse_result {
                cortex_m::interrupt::free(|cs| {
                    let mut buffer_ref = unsafe { GPS_SENTENCE_BUFFER.borrow(cs) }
                        .borrow_mut();
                    let buffer = buffer_ref.as_mut()
                        .unwrap();
                    match buffer.push_back(parse_result) {
                        Ok(()) => {},
                        Err(parse_result) => {
                            get_logger().log_str("warn: GPS buffer full. packet discarded.");
                            buffer.pop_front();
                            buffer.push_back(parse_result);
                        }
                    }
                });
            }
        }
        bytes = crate::gps::rx(&mut rx_buf).await;
    }
}

// Interrupt for radio DMA RX.
#[interrupt]
fn DMA1_STREAM5() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = unsafe { RX_TRANSFER.borrow(cs) }.borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        // Its important to clear fifo errors as the transfer is paused until it is cleared
        if Stream5::<pac::DMA1>::get_fifo_error_flag() {
            transfer.clear_fifo_error_interrupt();
        }
        if Stream5::<pac::DMA1>::get_transfer_complete_flag() {
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
fn USART2() {
    cortex_m::interrupt::free(|cs| {
        let mut transfer_ref = unsafe { RX_TRANSFER.borrow(cs) }.borrow_mut();
        let transfer = transfer_ref.as_mut().unwrap();

        let bytes = RX_BUFFER_SIZE as u16 - Stream5::<pac::DMA1>::get_number_of_transfers();
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

async fn gps_tx_complete() {
    loop {
        if TX_COMPLETE.load(Ordering::SeqCst) {
            break;
        }

        YieldFuture::new().await;
    }
}

async fn gps_rx_complete() {
    loop {
        if RX_COMPLETE.load(Ordering::SeqCst) {
            break;
        }

        YieldFuture::new().await;
    }
}

pub async fn tx(tx_buf: &[u8]) {
    gps_tx_complete().await;
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
    gps_tx_complete().await;
}

pub async fn rx(rx_buf: &mut [u8]) -> usize {
    gps_rx_complete().await;
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
        hprintln!("{:?}", core::str::from_utf8(&rx_buf[..bytes_copied]));
        bytes_copied
    })
}

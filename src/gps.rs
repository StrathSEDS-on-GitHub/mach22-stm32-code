use core::{
    cell::RefCell,
    cmp::min,
    sync::atomic::{AtomicBool, AtomicU32, AtomicU8, AtomicUsize, Ordering},
};

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use hal::{
    dma::{Stream6, StreamsTuple},
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

use crate::futures::YieldFuture;
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

const RX_BUFFER_SIZE: usize = 512;
pub fn setup(
    dma1: pac::DMA1,
    radio: hal::serial::Serial<
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
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM6);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_STREAM5);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
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
        for parse_result in parser.parse_from_bytes(&rx_buf[..bytes]) {
            hprintln!("{:?}", parse_result);
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

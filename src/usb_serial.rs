use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprintln;
use stm32f4xx_hal::{
    gpio::{Input, Pin},
    otg_fs::{UsbBus, USB},
    pac::{OTG_FS_DEVICE, OTG_FS_GLOBAL, OTG_FS_PWRCLK},
    rcc::Clocks,
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_serial::SerialPort;

use crate::futures::UsbFuture;

static mut SERIAL: Option<Serial> = None;
static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;
static mut EP_MEMORY: [u32; 1024] = [0; 1024];

/// Get a reference to the serial port.
pub fn get_serial() -> &'static mut Serial<'static> {
    // SAFETY: SERIAL is only mutated once at initialization.
    unsafe { SERIAL.as_mut().unwrap() }
}

/// Try to get a reference to the serial port
pub fn try_get_serial() -> Option<&'static Serial<'static>> {
    // SAFETY: SERIAL is only mutated once at initialization.
    unsafe { SERIAL.as_ref() }
}

/// A wrapper around the serial port that hides some unsafe stuff.
pub struct Serial<'a> {
    serial_port: Mutex<RefCell<SerialPort<'a, UsbBus<USB>>>>,
    usb_device: Mutex<RefCell<UsbDevice<'a, UsbBus<USB>>>>,
}

impl<'a> Serial<'a> {
    fn new(
        serial_port: SerialPort<'a, UsbBus<USB>>,
        usb_device: UsbDevice<'a, UsbBus<USB>>,
    ) -> Self {
        let serial_port = Mutex::new(RefCell::new(serial_port));
        let usb_device = Mutex::new(RefCell::new(usb_device));
        Serial {
            serial_port,
            usb_device,
        }
    }

    pub async fn write(&self, buf: &[u8]) {
        let mut write_offset = 0;
        let count = buf.len();

        UsbFuture::new(
            || {
                cortex_m::interrupt::free(|cs| {
                    let mut serial_port = self.serial_port.borrow(cs).borrow_mut();
                    while write_offset < count {
                        match serial_port.write(&buf[write_offset..count]) {
                            Ok(len) => {
                                if len > 0 {
                                    write_offset += len;
                                    //hprintln!("Wrote {} bytes", len);
                                };
                            }
                            Err(e) => return Err(e),
                        }
                    }
                    serial_port.flush()
                })
            },
            || self.poll(),
        )
        .await
        .unwrap();
    }

    pub async fn read(&self, buffer: &mut [u8]) -> Result<usize, UsbError> {
        UsbFuture::new(
            || {
                let s = cortex_m::interrupt::free(|cs| {
                    let mut serial_port = self.serial_port.borrow(cs).borrow_mut();
                    serial_port.read(buffer)
                });
                s
            },
            || self.poll(),
        )
        .await
    }

    pub fn read_no_block(&self, buffer: &mut [u8]) -> Result<usize, UsbError> {
        cortex_m::interrupt::free(|cs| {
            let mut serial_port = self.serial_port.borrow(cs).borrow_mut();
            serial_port.read(buffer)
        })
    }

    pub fn poll(&self) -> bool {
        cortex_m::interrupt::free(|cs| {
            let mut serial_port = self.serial_port.borrow(cs).borrow_mut();
            let mut usb_dev = self.usb_device.borrow(cs).borrow_mut();
            usb_dev.poll(&mut [&mut *serial_port])
        })
    }
}

impl core::fmt::Write for Serial<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let future = self.write(s.as_bytes());
        cassette::pin_mut!(future);
        let mut cm = cassette::Cassette::new(future);
        loop {
            if let Some(_) = cm.poll_on() {
                break;
            }
        }
        Ok(())
    }
}

pub fn setup_usb<'a>(
    otg_fs_global: OTG_FS_GLOBAL,
    otg_fs_device: OTG_FS_DEVICE,
    otg_fs_powerclock: OTG_FS_PWRCLK,
    gpioa_pa11: Pin<'A', 11, Input>,
    gpioa_pa12: Pin<'A', 12, Input>,
    clocks: &'a Clocks,
) {
    let usb = USB {
        usb_global: otg_fs_global,
        usb_device: otg_fs_device,
        usb_pwrclk: otg_fs_powerclock,
        pin_dm: gpioa_pa11.into_alternate(),
        pin_dp: gpioa_pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });
    // SAFETY: This function is the only access of USB_BUS and it is only called once at init.
    // As a result we can use static mut.
    unsafe {
        USB_BUS = Some(usb_bus);
    }

    let serial = usbd_serial::SerialPort::new(unsafe { &USB_BUS.as_ref().unwrap() });

    let usb_dev = UsbDeviceBuilder::new(
        unsafe { &USB_BUS.as_ref().unwrap() },
        UsbVidPid(0x16c0, 0x27dd),
    )
    .manufacturer("Fake company")
    .product("Serial port")
    .serial_number("TEST")
    .device_class(usbd_serial::USB_CLASS_CDC)
    .build();

    unsafe { SERIAL = Some(Serial::new(serial, usb_dev)) };
}

#![feature(async_closure)]
//! Demonstrate the use of a blocking `Delay` using the SYST (sysclock) timer.

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use crate::futures::NbFuture;
use crate::hal::timer::TimerExt;
use bmp388::BMP388;
use cassette::pin_mut;
use cassette::Cassette;
use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprint;
use cortex_m_semihosting::hprintln;
use hal::gpio::Pin;
use hal::i2c::I2c;
use hal::sdio::ClockFreq;
use hal::sdio::SdCard;
use hal::sdio::Sdio;
use hal::timer::CounterMs;
use libm::pow;

use core::cell::RefCell;
use core::panic::PanicInfo;
use hal::gpio;
use hal::gpio::Output;
use hal::gpio::PushPull;

use crate::hal::{pac, prelude::*};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

mod futures;
// SAFETY: It's properly Mutex'd and RefCell'd, so everything should be fine
static mut ERROR_LED: Option<Mutex<RefCell<gpio::Pin<'C', 13, Output<PushPull>>>>> = None;

#[entry]
fn entry_point() -> ! {
    let x = main();
    pin_mut!(x);
    let mut cm = Cassette::new(x);
    loop {
        if let Some(_) = cm.poll_on() {
            break;
        }
    }
    loop {}
}

async fn main() {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        // Set up the LED. On the Nucleo-446RE it's connected to pin PA5.
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        unsafe {
            ERROR_LED = Some(Mutex::new(RefCell::new(
                dp.GPIOC.split().pc13.into_push_pull_output(),
            )));
        }

        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(16.MHz())
            .sysclk(96.MHz())
            .require_pll48clk()
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .freeze();

        assert!(clocks.is_pll48clk_valid());

        let mut delay = cp.SYST.delay(&clocks);

        let d0 = gpiob.pb4.into_alternate().internal_pull_up(true);
        let d1 = gpioa.pa8.into_alternate().internal_pull_up(true);
        let d2 = gpioa.pa9.into_alternate().internal_pull_up(true);
        let d3 = gpiob.pb5.into_alternate().internal_pull_up(true);
        let clk = gpiob.pb15.into_alternate().internal_pull_up(false);
        let cmd = gpioa.pa6.into_alternate().internal_pull_up(true);
        let mut sdio: Sdio<SdCard> = Sdio::new(dp.SDIO, (clk, cmd, d0, d1, d2, d3), &clocks);

        hprintln!("Waiting for card...");

        // Wait for card to be ready
        loop {
            match sdio.init(ClockFreq::F24Mhz) {
                Ok(_) => break,
                Err(_err) => (),
            }

            delay.delay_ms(1000u32);
        }

        let nblocks = sdio.card().map(|c| c.block_count()).unwrap_or(0);
        hprintln!("Card detected: nbr of blocks: {:?}", nblocks);

        // Read a block from the card and print the data
        let mut block = [0u8; 512];

        match sdio.read_block(0, &mut block) {
            Ok(()) => (),
            Err(err) => {
                hprintln!("Failed to read block: {:?}", err);
            }
        }

        for b in block.iter() {
            hprint!("{:X} ", b);
        }
        
        let timer = dp.TIM2.counter_ms(&clocks);
        let i2c = I2c::new(dp.I2C2, (gpiob.pb10, gpiob.pb9), 100.kHz(), &clocks);
        let f1 = report_sensor_data(i2c, timer);
        let timer = dp.TIM3.counter_ms(&clocks);
        let f2 = blink_led(gpioa.pa4.into_push_pull_output(), timer);

        ::futures::join!(f2, f1);
    }
}

async fn blink_led<const P: char, const T: u8>(
    mut led: Pin<P, T, Output<PushPull>>,
    mut timer: CounterMs<pac::TIM3>,
) {
    timer.start(500.millis()).unwrap();
    loop {
        led.set_high();
        NbFuture::new(|| timer.wait()).await.unwrap();
        led.set_low();
        NbFuture::new(|| timer.wait()).await.unwrap();
    }
}

async fn report_sensor_data(
    i2c: I2c<pac::I2C2, (gpio::Pin<'B', 10>, gpio::Pin<'B', 9>)>,
    mut timer: CounterMs<pac::TIM2>,
) {
    let mut bmp388 = BMP388::new(i2c).unwrap();
    bmp388
        .set_power_control(bmp388::PowerControl {
            pressure_enable: true,
            temperature_enable: true,
            mode: bmp388::PowerMode::Normal,
        })
        .unwrap();
    timer.start(5000.millis()).unwrap();
    loop {
        bmp388
            .sensor_values()
            .map(|values| {
                let alt = (1.0 / 0.0065)
                    * (pow((values.pressure) / 101325.0, 1.0 / 5.257) - 1.0)
                    * values.temperature;

                hprintln!(
                    "Temperature: {:.2}°C, Pressure: {:.2}Pa, Altitude: {:.2}m",
                    values.temperature,
                    values.pressure,
                    alt
                );
            })
            .unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
    }
}

#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    unsafe {
        if let Some(led) = ERROR_LED.as_mut() {
            cortex_m::interrupt::free(|cs| {
                let mut led = led.borrow(cs).borrow_mut();
                led.set_high();
            });
            hprintln!("Panic! {}", info);
        }
    }
    loop {}
}

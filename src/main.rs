#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use crate::hal::timer::TimerExt;
use crate::mission::MissionState;
use crate::rickroll::rickroll_everyone;
use crate::rickroll::RickRollPeripherals;
use crate::rickroll::RICK_ROLL_PERIPHERALS;
use crate::sdcard::get_logger;
use crate::sdcard::SdLogger;
use crate::usb_serial::setup_usb;
use ::futures::join;
use bmp388::BMP388;
use cassette::pin_mut;
use cassette::Cassette;
use cortex_m::interrupt::Mutex;
use cortex_m_semihosting::hprint;
use cortex_m_semihosting::hprintln;
use embedded_hal::digital::v2::OutputPin;
use hal::gpio::Edge;
use hal::gpio::Input;
use hal::gpio::PinState;
use hal::i2c::I2c;
use hal::interrupt;
use hal::rcc::Clocks;
use hal::rtc::Rtc;
use hal::sdio::ClockFreq;
use hal::sdio::SdCard;
use hal::sdio::Sdio;

use core::cell::RefCell;
use core::panic::PanicInfo;
use hal::gpio;
use hal::gpio::Output;
use hal::gpio::PushPull;

use crate::hal::{pac, prelude::*};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
mod bmi055;
mod futures;
mod gps;
mod mission;
mod radio;
mod rickroll;
mod sdcard;
mod usb_serial;

static mut ERROR_LED: Mutex<RefCell<Option<gpio::Pin<'C', 13, Output<PushPull>>>>> =
    Mutex::new(RefCell::new(None));
static mut USER_BUTTON: Mutex<RefCell<Option<gpio::Pin<'A', 0, Input>>>> =
    Mutex::new(RefCell::new(None));

static mut RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));

fn get_timestamp() -> i64 {
    cortex_m::interrupt::free(|cs| {
        let mut rtc_ref = unsafe { crate::RTC.borrow(cs) }.borrow_mut();
        rtc_ref
            .as_mut()
            .unwrap()
            .get_datetime()
            .assume_utc()
            .unix_timestamp()
    })
}

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
static mut CLOCKS: Mutex<RefCell<Option<Clocks>>> = Mutex::new(RefCell::new(None));

async fn main() {
    if let (Some(mut dp), Some(mut cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();

        cortex_m::interrupt::free(|cs| {
            // SAFETY: Mutex makes access of static mutable variable safe
            unsafe { ERROR_LED.borrow(cs) }
                .replace(Some(dp.GPIOC.split().pc13.into_push_pull_output()));
        });

        dp.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());

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

        let mut button = gpioa.pa0.into_pull_up_input();

        let mut syscfg = dp.SYSCFG.constrain();
        button.make_interrupt_source(&mut syscfg);
        button.enable_interrupt(&mut dp.EXTI);
        button.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
        let btn_int_num = button.interrupt();
        pac::NVIC::unpend(btn_int_num);

        let counter = dp.TIM5.counter_us(&clocks);
        let pin = gpioa.pa1.into_alternate();
        let led = gpioa
            .pa5
            .internal_pull_down(true)
            .into_push_pull_output_in_state(PinState::Low);
        let pwm = dp.TIM2.pwm_hz(pin, (96 / 4).MHz(), &clocks).split();

        // SAFETY: We are not in a CS so we can safely unmask interrupts.
        unsafe {
            pac::NVIC::unmask(btn_int_num);
        };

        cortex_m::interrupt::free(|cs| {
            // SAFETY: Mutex makes access of static mutable variable safe
            unsafe { USER_BUTTON.borrow(cs) }.replace(Some(button));
            unsafe { RICK_ROLL_PERIPHERALS.borrow(cs) }
                .replace(Some(RickRollPeripherals::new(led, counter, pwm)));
        });

        if matches!(mission::NODE_TYPE, mission::NodeType::GroundStation(..)) {
            setup_usb(
                dp.OTG_FS_GLOBAL,
                dp.OTG_FS_DEVICE,
                dp.OTG_FS_PWRCLK,
                gpioa.pa11,
                gpioa.pa12,
                &clocks,
            );
        }

        let mut delay = cp.SYST.delay(&clocks);

        if !matches!(mission::NODE_TYPE, mission::NodeType::GroundStation(_)) {
            let d0 = gpiob.pb4.into_alternate().internal_pull_up(true);
            let clk = gpiob.pb15.into_alternate().internal_pull_up(false);
            let cmd = gpioa.pa6.into_alternate().internal_pull_up(true);
            let mut sdio: Sdio<SdCard> = Sdio::new(dp.SDIO, (clk, cmd, d0), &clocks);

            hprintln!("Waiting for card...");

            // Wait for card to be ready
            loop {
                match sdio.init(ClockFreq::F8Mhz) {
                    Ok(_) => break,
                    Err(_err) => (hprint!("_err {:?}\n", _err)),
                }

                delay.delay_ms(1000u32);
            }

            let nblocks = sdio.card().map(|c| c.block_count()).unwrap_or(0);
            hprintln!("Card detected: nbr of blocks: {:?}", nblocks);

            SdLogger::new(sdio).expect("Failed to create SD logger");
        }
        let rtc = Rtc::new(dp.RTC, &mut dp.PWR);
        cortex_m::interrupt::free(|cs| {
            // SAFETY: Mutex makes access of static mutable variable safe
            unsafe { RTC.borrow(cs).replace(Some(rtc)) }
        });


        // let mut i2c1 = I2c::new(dp.I2C1, (gpiob.pb6, gpiob.pb7), 100.kHz(), &clocks);
        let i2c2 = I2c::new(dp.I2C2, (gpiob.pb10, gpiob.pb9), 100.kHz(), &clocks);
        let timer = dp.TIM3.counter_ms(&clocks);

        let radio = dp
            .USART1
            .serial(
                (gpioa.pa15.into_alternate(), gpioa.pa10.into_alternate()),
                hal::serial::config::Config {
                    baudrate: 9600.bps(),
                    dma: hal::serial::config::DmaConfig::TxRx,
                    ..Default::default()
                },
                &clocks,
            )
            .unwrap();

        radio::setup(dp.DMA2, radio);

        let gps = dp
            .USART2
            .serial(
                (gpioa.pa2.into_alternate(), gpioa.pa3.into_alternate()),
                hal::serial::config::Config {
                    baudrate: 9600.bps(),
                    dma: hal::serial::config::DmaConfig::TxRx,
                    ..Default::default()
                },
                &clocks,
            )
            .unwrap();

        gps::setup(dp.DMA1, gps);

        cortex_m::interrupt::free(|cs| {
            // SAFETY: Mutex makes access of static mutable variable safe
            unsafe { CLOCKS.borrow(cs) }.replace(Some(clocks));
        });

        let mut bmp388 = BMP388::new(i2c2).unwrap();
        bmp388
            .set_power_control(bmp388::PowerControl {
                pressure_enable: true,
                temperature_enable: true,
                mode: bmp388::PowerMode::Normal,
            })
            .unwrap();

        let f2 = radio::parse_recvd_data();
        let f3 = gps::parse_recvd_data();

        let mut mission = MissionState::new();
        join!(f2, f3, mission.start(bmp388, timer));
    }
}

#[panic_handler]
pub fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::free(|cs| {
        // SAFETY: Mutex makes access of static mutable variable safe
        let mut led = unsafe { ERROR_LED.borrow(cs).borrow_mut() };
        if let Some(led) = led.as_mut() {
            led.set_high();
        }
    });
    hprintln!("Panic! {}", info);
    get_logger().log(format_args!("Panic! {}", info));
    loop {}
}

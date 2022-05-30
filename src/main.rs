#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use crate::bmi055::BMI055;
use crate::futures::NbFuture;
use crate::hal::timer::TimerExt;
use crate::usb_serial::setup_usb;
use bmp388::BMP388;
use cassette::pin_mut;
use cassette::Cassette;
use cortex_m::interrupt::Mutex;
use cortex_m::iprintln;
use cortex_m::itm;
use cortex_m::peripheral::itm::Stim;
use cortex_m_semihosting::hprint;
use cortex_m_semihosting::hprintln;
use embedded_hal::PwmPin;
use embedded_hal::digital::v2::OutputPin;
use hal::pac::USART1;
use hal::pac::USART2;
use hal::serial::Rx;
use hal::serial::Serial;
use hal::serial::Tx;
use usb_device::UsbError;
use usb_serial::get_serial;
use crate::futures::YieldFuture;
use hal::gpio::Edge;
use hal::gpio::Input;
use hal::gpio::Pin;
use hal::i2c::I2c;
use hal::interrupt;
use hal::pac::DBGMCU;
use hal::pac::TIM2;
use hal::pac::TIM5;
use hal::timer;
use hal::timer::CounterHz;
use hal::timer::DelayMs;
use hal::timer::PwmChannel;
use libm::pow;
use libm::sin;

use core::cell::RefCell;
use core::cmp::max;
use core::cmp::min;
use core::f32::consts::FRAC_PI_2;
use core::fmt::Debug;
use core::fmt::Write;
use core::panic::PanicInfo;
use core::ptr;
use hal::gpio;
use hal::gpio::Output;
use hal::gpio::PushPull;

use crate::hal::{pac, prelude::*};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
mod futures;
mod bmi055;
mod usb_serial;

const TEMPO: i32 = 114;
const MELODY: &[i32] = &[
  587,-4, 659,-4, 440,4,
  659,-4, 740,-4, 880,16, 784,16, 740,8,
  587,-4, 659,-4, 440,2,
  440,16, 440,16, 494,16, 587,8, 587,16,
  587,-4, 659,-4, 440,4,
  659,-4, 740,-4, 880,16, 784,16, 740,8,
  587,-4, 659,-4, 440,2,
  440,16, 440,16, 494,16, 587,8, 587,16,
  0,4, 494,8, 554,8, 587,8, 587,8, 659,8, 554,-8,
  494,16, 440,2, 0,4,

  0,8, 494,8, 494,8, 554,8, 587,8, 494,4, 440,8,
  880,8, 0,8, 880,8, 659,-4, 0,4,
  494,8, 494,8, 554,8, 587,8, 494,8, 587,8, 659,8, 0,8,
  0,8, 554,8, 494,8, 440,-4, 0,4,
  0,8, 494,8, 494,8, 554,8, 587,8, 494,8, 440,4,
  659,8, 659,8, 659,8, 740,8, 659,4, 0,4,

  587,2, 659,8, 740,8, 587,8,
  659,8, 659,8, 659,8, 740,8, 659,4, 440,4,
  0,2, 494,8, 554,8, 587,8, 494,8,
  0,8, 659,8, 740,8, 659,-4, 440,16, 494,16, 587,16, 494,16,
  740,-8, 740,-8, 659,-4, 440,16, 494,16, 587,16, 494,16,

  659,-8, 659,-8, 587,-8, 554,16, 494,-8, 440,16, 494,16, 587,16, 494,16,
  587,4, 659,8, 554,-8, 494,16, 440,8, 440,8, 440,8,
  659,4, 587,2, 440,16, 494,16, 587,16, 494,16,
  740,-8, 740,-8, 659,-4, 440,16, 494,16, 587,16, 494,16,
  880,4, 554,8, 587,-8, 554,16, 494,8, 440,16, 494,16, 587,16, 494,16,

  587,4, 659,8, 554,-8, 494,16, 440,4, 440,8,
  659,4, 587,2, 0,4,
  0,8, 494,8, 587,8, 494,8, 587,8, 659,4, 0,8,
  0,8, 554,8, 494,8, 440,-4, 0,4,
  0,8, 494,8, 494,8, 554,8, 587,8, 494,8, 440,4,
  0,8, 880,8, 880,8, 659,8, 740,8, 659,8, 587,8,

  0,8, 440,8, 494,8, 554,8, 587,8, 494,8,
  0,8, 554,8, 494,8, 440,-4, 0,4,
  494,8, 494,8, 554,8, 587,8, 494,8, 440,4, 0,8,
  0,8, 659,8, 659,8, 740,4, 659,-4,
  587,2, 587,8, 659,8, 740,8, 659,4,
  659,8, 659,8, 740,8, 659,8, 440,8, 440,4,

  0,-4, 440,8, 494,8, 554,8, 587,8, 494,8,
  0,8, 659,8, 740,8, 659,-4, 440,16, 494,16, 587,16, 494,16,
  740,-8, 740,-8, 659,-4, 440,16, 494,16, 587,16, 494,16,
  659,-8, 659,-8, 587,-8, 554,16, 494,8, 440,16, 494,16, 587,16, 494,16,
  587,4, 659,8, 554,-8, 494,16, 440,4, 440,8,

   659,4, 587,2, 440,16, 494,16, 587,16, 494,16,
  740,-8, 740,-8, 659,-4, 440,16, 494,16, 587,16, 494,16,
  880,4, 554,8, 587,-8, 554,16, 494,8, 440,16, 494,16, 587,16, 494,16,
  587,4, 659,8, 554,-8, 494,16, 440,4, 440,8,
  659,4, 587,2, 440,16, 494,16, 587,16, 494,16,

  740,-8, 740,-8, 659,-4, 440,16, 494,16, 587,16, 494,16,
  880,4, 554,8, 587,-8, 554,16, 494,8, 440,16, 494,16, 587,16, 494,16,
  587,4, 659,8, 554,-8, 494,16, 440,4, 440,8,
  659,4, 587,2, 440,16, 494,16, 587,16, 494,16,
  740,-8, 740,-8, 659,-4, 440,16, 494,16, 587,16, 494,16,

  880,4, 554,8, 587,-8, 554,16, 494,8, 440,16, 494,16, 587,16, 494,16,
  587,4, 659,8, 554,-8, 494,16, 440,4, 440,8,

  659,4, 587,2, 0,4
];

static mut ERROR_LED: Mutex<RefCell<Option<gpio::Pin<'C', 13, Output<PushPull>>>>> =
    Mutex::new(RefCell::new(None));
static mut USER_BUTTON: Mutex<RefCell<Option<gpio::Pin<'A', 0, Input>>>> =
    Mutex::new(RefCell::new(None));

static mut SHOULD_RICKROLL: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

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
    if let (Some(mut dp), Some(mut cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();

        cortex_m::interrupt::free(|cs| {
            // SAFETY: Mutex makes access of static mutable variable safe
            unsafe { ERROR_LED.borrow(cs) }.replace(Some(dp.GPIOC.split().pc13.into_push_pull_output()));
        });

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

        setup_usb(dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK, gpioa.pa11, gpioa.pa12, &clocks);

        /* let mut delay = cp.SYST.delay(&clocks);

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
        } */

        let mut syscfg = dp.SYSCFG.constrain();
        let mut button = gpioa.pa0.into_pull_up_input();
        button.make_interrupt_source(&mut syscfg);
        button.enable_interrupt(&mut dp.EXTI);
        button.trigger_on_edge(&mut dp.EXTI, Edge::RisingFalling);
        let btn_int_num = button.interrupt();
        pac::NVIC::unpend(btn_int_num);

        // SAFETY: We are not in a CS so we can safely unmask interrupts.
        unsafe {
            pac::NVIC::unmask(btn_int_num);
        };

        cortex_m::interrupt::free(|cs| {
            // SAFETY: Mutex makes access of static mutable variable safe
            unsafe { USER_BUTTON.borrow(cs) }.replace(Some(button));
        });
        
        let counter = dp.TIM5.counter_hz(&clocks);
        let pin = gpioa.pa1.into_alternate();
        let led = gpioa.pa5.into_push_pull_output();
        let pwm = dp.TIM2.pwm_hz(pin, 100.kHz(), &clocks).split();
        

        let timer = dp.TIM4.counter_ms(&clocks);
        let i2c1 = I2c::new(dp.I2C1, (gpiob.pb6, gpiob.pb7), 100.kHz(), &clocks);
        let i2c2 = I2c::new(dp.I2C2, (gpiob.pb10, gpiob.pb9), 100.kHz(), &clocks);
        //let f1 = report_sensor_data(i2c1, i2c2, timer);
        let timer = dp.TIM3.delay_ms(&clocks);

        let radio = dp.USART2.serial((gpioa.pa2.into_alternate(), gpioa.pa3.into_alternate()), 9600.bps(), &clocks).unwrap();
        let gps = dp.USART1.serial((gpioa.pa15.into_alternate(), gpioa.pa10.into_alternate()), 9600.bps(), &clocks).unwrap();

        let f2 = radio_serial(radio, gps, led, timer);
        //let f3 = rickroll_everyone(pwm, counter, led);

        f2.await;
    }
}

async fn rickroll_everyone<const C: u8, LedPin: OutputPin, E: Debug>(mut pwm: timer::PwmChannel<pac::TIM2, C>, mut counter: timer::CounterHz<pac::TIM5>, mut led: LedPin) 
where LedPin: OutputPin<Error = E> {
    let chunks = MELODY.chunks(2);
    let note_length_ms: i32 = 60000 * 4 / TEMPO;
    pwm.enable();
    for note_and_duration in chunks {
        loop {
            let should_rickroll = cortex_m::interrupt::free(|cs| { 
                // SAFETY: Mutex makes access of static mutable variable safe
                unsafe { *SHOULD_RICKROLL.borrow(cs).borrow() }
            });
            if should_rickroll {
                break;
            } else {
                YieldFuture::new().await;
            }
        }

        let (note, divider) = (note_and_duration[0], note_and_duration[1]);
        // calculates the duration of each note
        let note_duration = if divider > 0 {
          // regular note, just proceed
            (note_length_ms) / divider
        } else {
          // dotted notes are represented with negative durations!!
          (note_length_ms)  * 3 / (-divider) / 2
        };


        if note == 0 {
            counter.start(max((1_000/note_duration) as u32, 1).kHz()).unwrap();
            NbFuture::new(|| counter.wait()).await.unwrap();
            continue;
        }

        counter.start(((note * 2) as u32).Hz()).unwrap();
        let mut pulse = false;
        let mut n = 0;
        loop {
            if pulse {
                pwm.set_duty(pwm.get_max_duty());
                led.set_high().unwrap();
            } else {
                pwm.set_duty(0);
                led.set_low().unwrap();
            }
            NbFuture::new(|| counter.wait()).await.unwrap();
            pulse = if 1000 * n / (note * 2) >= note_duration * 8 / 10 { false } else { !pulse };
            n += 1;

            if 1000 * n / (note * 2) >= note_duration {
                break;
            }
        }
    }
}

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| 
        // SAFETY: Mutex makes access of static mutable variable safe
        {
            let mut btn_ref = unsafe { USER_BUTTON.borrow(cs) }.borrow_mut();
            let btn = btn_ref.as_mut().unwrap();
            btn.clear_interrupt_pending_bit();
            let mut should_rickroll = unsafe { SHOULD_RICKROLL.borrow(cs) }.borrow_mut();
            *should_rickroll = btn.is_low();
    });
}

async fn radio_serial<TIM, PinsRadio, PinsGPS, const P: char, const T: u8>(
    mut radio: Serial<USART2, PinsRadio, u8>,
    mut gps: Serial<USART1, PinsGPS, u8>,
    mut led: Pin<P, T, Output<PushPull>>,
    mut timer: DelayMs<TIM>,
) where
    TIM: timer::Instance,
{

    nb::block!(radio.write('+' as u8)).unwrap();
    nb::block!(radio.write('+' as u8)).unwrap();
    nb::block!(radio.write('+' as u8)).unwrap();

    check_radio_ok(&mut radio);

    let command = b"ATAP \x02\r";
    for b in command {
        nb::block!(radio.write(*b)).unwrap();
    }
    check_radio_ok(&mut radio);

    let command = b"CN\r";
    for b in command {
        nb::block!(radio.write(*b)).unwrap();
    }

    let bytes_read = 0;
    loop {
        if let Ok(byte) = radio.read() {
            bytes_read += 1;
            get_serial().write(&[byte]).await;
        }
        if bytes_read % 17 == 0 {
            let data = [0x7Eu8, 0x00, 0xFF, 0x08, 0x01, 'D' as u8, 'B' as u8, 0x00];
            let checksum = data[3..].iter().fold(0, |acc, &x| (acc as u8).wrapping_add(x));
            data.last().replace(&checksum);

            for byte in data { 
                nb::block!(radio.write(byte)).unwrap();
            }

            let mut resp_bytes = 0;
            loop {
                if let Ok(byte) = radio.read() {
                    get_serial().write(&[byte]).await;
                    resp_bytes += 1;
                    if resp_bytes == 10 {
                        break;
                    }
                }
            }
        }
        get_serial().poll();
    }


}

fn check_radio_ok<PinsRadio>(radio: &mut Serial<USART2, PinsRadio>) {
    let mut data = [0u8;3];
    let mut bytes = 0;
    loop {
        match radio.read() {
            Ok(b) => {
                data[bytes] = b;
                bytes += 1;
                if bytes == 3 { break }
            }
            Err(nb::Error::WouldBlock) =>{}
            Err(nb::Error::Other(e)) => {
                panic!("{:?}", e);
            }
        }
    }
    if &data != b"OK\r" { panic!("Reply not OK."); }
}

async fn report_sensor_data<TIM, PINS>(
    i2c1: I2c<pac::I2C1, PINS>,
    i2c2: I2c<pac::I2C2, (gpio::Pin<'B', 10>, gpio::Pin<'B', 9>)>,
    mut timer: CounterMs<TIM>,
) where
    TIM: timer::Instance,
{
    let mut bmp388 = BMP388::new(i2c2).unwrap();
    bmp388
        .set_power_control(bmp388::PowerControl {
            pressure_enable: true,
            temperature_enable: true,
            mode: bmp388::PowerMode::Normal,
        })
        .unwrap();


    //let mut bmi055 = BMI055::new(i2c1).unwrap();
    //hprintln!("BMI055: {:?}", bmi055.id().unwrap());

    timer.start(5000.millis()).unwrap();
    loop {
        bmp388
            .sensor_values()
            .map(|values| {
                let alt = (1.0 / 0.0065)
                    * (pow((values.pressure) / 101325.0, 1.0 / 5.257) - 1.0)
                    * values.temperature;

                /*hprintln!(
                    "Temperature: {:.2}°C, Pressure: {:.2}Pa, Altitude: {:.2}m",
                    values.temperature,
                    values.pressure,
                    alt
                );*/
            })
            .unwrap();
        NbFuture::new(|| timer.wait()).await.unwrap();
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
        // hprintln!("Panic! {}", info);
    loop {}
}

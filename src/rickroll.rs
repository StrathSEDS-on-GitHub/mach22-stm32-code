use core::{cell::RefCell, cmp::max};

use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
    gpio::{ErasedPin, Output, Pin},
    pac,
    prelude::{_fugit_RateExtU32, _fugit_DurationExtU32},
    timer::{self, CounterUs, Pwm, PwmChannel},
};

use crate::futures::NbFuture;

pub struct RickRollPeripherals {
    led: Pin<'A', 5, Output>,
    counter: CounterUs<pac::TIM5>,
    pwm: PwmChannel<pac::TIM2, 1>,
}
impl RickRollPeripherals {
    pub fn new(led: Pin<'A', 5, Output>, counter: CounterUs<pac::TIM5>, pwm: PwmChannel<pac::TIM2, 1>) -> Self {
        Self { led, counter, pwm }
    }
}

pub static mut RICK_ROLL_PERIPHERALS: Mutex<RefCell<Option<RickRollPeripherals>>> = Mutex::new(RefCell::new(None));

const TEMPO: i32 = 114;
const MELODY: &[i32] = &[
    587, -4, 659, -4, 440, 4, 659, -4, 740, -4, 880, 16, 784, 16, 740, 8, 587, -4, 659, -4, 440, 2,
    440, 16, 440, 16, 494, 16, 587, 8, 587, 16, 587, -4, 659, -4, 440, 4, 659, -4, 740, -4, 880,
    16, 784, 16, 740, 8, 587, -4, 659, -4, 440, 2, 440, 16, 440, 16, 494, 16, 587, 8, 587, 16, 0,
    4, 494, 8, 554, 8, 587, 8, 587, 8, 659, 8, 554, -8, 494, 16, 440, 2, 0, 4, 0, 8, 494, 8, 494,
    8, 554, 8, 587, 8, 494, 4, 440, 8, 880, 8, 0, 8, 880, 8, 659, -4, 0, 4, 494, 8, 494, 8, 554, 8,
    587, 8, 494, 8, 587, 8, 659, 8, 0, 8, 0, 8, 554, 8, 494, 8, 440, -4, 0, 4, 0, 8, 494, 8, 494,
    8, 554, 8, 587, 8, 494, 8, 440, 4, 659, 8, 659, 8, 659, 8, 740, 8, 659, 4, 0, 4, 587, 2, 659,
    8, 740, 8, 587, 8, 659, 8, 659, 8, 659, 8, 740, 8, 659, 4, 440, 4, 0, 2, 494, 8, 554, 8, 587,
    8, 494, 8, 0, 8, 659, 8, 740, 8, 659, -4, 440, 16, 494, 16, 587, 16, 494, 16, 740, -8, 740, -8,
    659, -4, 440, 16, 494, 16, 587, 16, 494, 16, 659, -8, 659, -8, 587, -8, 554, 16, 494, -8, 440,
    16, 494, 16, 587, 16, 494, 16, 587, 4, 659, 8, 554, -8, 494, 16, 440, 8, 440, 8, 440, 8, 659,
    4, 587, 2, 440, 16, 494, 16, 587, 16, 494, 16, 740, -8, 740, -8, 659, -4, 440, 16, 494, 16,
    587, 16, 494, 16, 880, 4, 554, 8, 587, -8, 554, 16, 494, 8, 440, 16, 494, 16, 587, 16, 494, 16,
    587, 4, 659, 8, 554, -8, 494, 16, 440, 4, 440, 8, 659, 4, 587, 2, 0, 4, 0, 8, 494, 8, 587, 8,
    494, 8, 587, 8, 659, 4, 0, 8, 0, 8, 554, 8, 494, 8, 440, -4, 0, 4, 0, 8, 494, 8, 494, 8, 554,
    8, 587, 8, 494, 8, 440, 4, 0, 8, 880, 8, 880, 8, 659, 8, 740, 8, 659, 8, 587, 8, 0, 8, 440, 8,
    494, 8, 554, 8, 587, 8, 494, 8, 0, 8, 554, 8, 494, 8, 440, -4, 0, 4, 494, 8, 494, 8, 554, 8,
    587, 8, 494, 8, 440, 4, 0, 8, 0, 8, 659, 8, 659, 8, 740, 4, 659, -4, 587, 2, 587, 8, 659, 8,
    740, 8, 659, 4, 659, 8, 659, 8, 740, 8, 659, 8, 440, 8, 440, 4, 0, -4, 440, 8, 494, 8, 554, 8,
    587, 8, 494, 8, 0, 8, 659, 8, 740, 8, 659, -4, 440, 16, 494, 16, 587, 16, 494, 16, 740, -8,
    740, -8, 659, -4, 440, 16, 494, 16, 587, 16, 494, 16, 659, -8, 659, -8, 587, -8, 554, 16, 494,
    8, 440, 16, 494, 16, 587, 16, 494, 16, 587, 4, 659, 8, 554, -8, 494, 16, 440, 4, 440, 8, 659,
    4, 587, 2, 440, 16, 494, 16, 587, 16, 494, 16, 740, -8, 740, -8, 659, -4, 440, 16, 494, 16,
    587, 16, 494, 16, 880, 4, 554, 8, 587, -8, 554, 16, 494, 8, 440, 16, 494, 16, 587, 16, 494, 16,
    587, 4, 659, 8, 554, -8, 494, 16, 440, 4, 440, 8, 659, 4, 587, 2, 440, 16, 494, 16, 587, 16,
    494, 16, 740, -8, 740, -8, 659, -4, 440, 16, 494, 16, 587, 16, 494, 16, 880, 4, 554, 8, 587,
    -8, 554, 16, 494, 8, 440, 16, 494, 16, 587, 16, 494, 16, 587, 4, 659, 8, 554, -8, 494, 16, 440,
    4, 440, 8, 659, 4, 587, 2, 440, 16, 494, 16, 587, 16, 494, 16, 740, -8, 740, -8, 659, -4, 440,
    16, 494, 16, 587, 16, 494, 16, 880, 4, 554, 8, 587, -8, 554, 16, 494, 8, 440, 16, 494, 16, 587,
    16, 494, 16, 587, 4, 659, 8, 554, -8, 494, 16, 440, 4, 440, 8, 659, 4, 587, 2, 0, 4,
];

pub async fn rickroll_everyone() {
    let (mut led, mut counter, mut pwm) = cortex_m::interrupt::free(|cs| unsafe {
        let p = RICK_ROLL_PERIPHERALS
            .borrow(cs)
            .borrow_mut()
            .take()
            .unwrap();
        (p.led, p.counter, p.pwm)
    });
    let chunks = MELODY.chunks(2);
    let note_length_ms: i32 = 60000 * 4 / TEMPO;
    pwm.enable();
    pwm.set_duty(pwm.get_max_duty()/2);
    for note_and_duration in chunks {
        let (note, divider) = (note_and_duration[0], note_and_duration[1]);
        // calculates the duration of each note
        let note_duration = if divider > 0 {
            // regular note, just proceed
            (note_length_ms) / divider
        } else {
            // dotted notes are represented with negative durations!!
            (note_length_ms) * 3 / (-divider) / 2
        };

        if note == 0 {
            pwm.disable();
            counter
                .start((note_duration as u32).millis())
                .unwrap();
            NbFuture::new(|| counter.wait()).await.unwrap();
            pwm.enable();
            continue;
        }


        // Default clock is 24MHz.
        let psc = (24_000_000 / note) as u16;
        let tim2 = unsafe { &*pac::TIM2::ptr() };
        tim2.psc.write(|w| w.psc().bits(psc));

        counter.start((note_duration as u32).millis()).unwrap();
        NbFuture::new(|| counter.wait()).await.unwrap();
    }
}

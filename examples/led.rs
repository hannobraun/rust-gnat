#![no_main]
#![no_std]


extern crate panic_halt;


use cortex_m_rt::entry;
use gnat::hal::{
    prelude::*,
    pac,
    rcc,
};


#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc   = dp.RCC.freeze(rcc::Config::hsi16());
    let     gpiob = dp.GPIOB.split(&mut rcc);
    let mut delay = cp.SYST.delay(rcc.clocks);

    let mut led = gpiob.pb12.into_push_pull_output();

    loop {
        led.set_low().unwrap();
        delay.delay_ms(100u16);

        led.set_high().unwrap();
        delay.delay_ms(600u16);
    }
}

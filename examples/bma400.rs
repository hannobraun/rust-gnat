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

    let mut i2c = dp
        .I2C1
        .i2c(
            gpiob.pb9.into_open_drain_output(), // SDA
            gpiob.pb8.into_open_drain_output(), // SCL
            100.khz(),
            &mut rcc,
        );

    let mut led = gpiob.pb12.into_push_pull_output();

    loop {
        let mut buffer = [0; 1];

        i2c
            .write_read(
                0b0010100, // device address
                &[0x00],   // CHIPID register
                &mut buffer,
            )
            .unwrap();

        if buffer[0] == 0x90 {
            // Device correctly identified. Blink to signal sucess.
            led.set_low().unwrap();
            delay.delay_ms(10u16);

            led.set_high().unwrap();
            delay.delay_ms(70u16);
        }
    }
}

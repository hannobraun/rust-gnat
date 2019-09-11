#![no_main]
#![no_std]


extern crate panic_halt;


use cortex_m_rt::entry;
use stm32l0xx_hal::{
    prelude::*,
    exti,
    pac,
    pwr::{
        self,
        PWR,
    },
    rcc::Config,
    syscfg::SYSCFG,
};


#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc    = dp.RCC.freeze(Config::hsi16());
    let     gpiob  = dp.GPIOB.split(&mut rcc);
    let mut exti   = dp.EXTI;
    let mut pwr    = PWR::new(dp.PWR, &mut rcc);
    let mut delay  = cp.SYST.delay(rcc.clocks);
    let mut scb    = cp.SCB;
    let mut syscfg = SYSCFG::new(dp.SYSCFG_COMP, &mut rcc);

    let     button = gpiob.pb5.into_floating_input();
    let mut led    = gpiob.pb12.into_push_pull_output();

    // Disable LED
    led.set_high().unwrap();

    exti.listen(
        &mut syscfg,
        button.port,
        button.i,
        exti::TriggerEdge::Rising,
    );

    loop {
        exti.wait_for_irq(
            button.i,
            pwr.stop_mode(
                &mut scb,
                &mut rcc,
                pwr::StopModeConfig {
                    ultra_low_power: true,
                },
            ),
        );

        led.set_low().unwrap();
        delay.delay_ms(100u32);
        led.set_high().unwrap();
    }
}

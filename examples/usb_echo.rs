#![no_main]
#![no_std]


extern crate panic_halt;


use cortex_m::{
    interrupt,
    peripheral::NVIC,
};
use cortex_m_rt::entry;
use gnat::hal::{
    prelude::*,
    pac::{
        self,
        Interrupt,
    },
    pwr::PWR,
    rcc,
    syscfg::SYSCFG,
    usb::USB,
};
use stm32_usbd::UsbBus;
use usbd_serial::{
    SerialPort,
    USB_CLASS_CDC,
};
use usb_device::{
    UsbError,
    bus::UsbBus as UsbBusTrait,
    device::{
        UsbDeviceBuilder,
        UsbVidPid,
    },
};


#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut scb    = cp.SCB;
    let mut rcc    = dp.RCC.freeze(rcc::Config::hsi16());
    let mut pwr    = PWR::new(dp.PWR, &mut rcc);
    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
    let     gpioa  = dp.GPIOA.split(&mut rcc);
    let     gpiob  = dp.GPIOB.split(&mut rcc);

    let mut led = gpiob.pb12.into_push_pull_output();
    led.set_high().unwrap(); // disable LED

    let usb_dm = gpioa.pa11;
    let usb_dp = gpioa.pa12;

    let hsi48 = rcc.enable_hsi48(&mut syscfg, dp.CRS);
    let usb   = USB::new(dp.USB, usb_dm, usb_dp, hsi48);

    let     bus    = UsbBus::new(usb);
    let mut serial = SerialPort::new(&bus);

    // Use special VID/PID for testing from pid.codes.
    // http://pid.codes/1209/0001/
    let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("MANUFACTURER")
        .product("PRODUCT")
        .serial_number("SERIAL_NUMBER")
        .device_class(USB_CLASS_CDC)
        .build();

    // If this program is run right after being uploaded via USB, the host
    // computer will be confused and still think it's connected to the
    // bootloader. Let's force it to recognize us, as to not require a manual
    // reset after each upload.
    device.bus().force_reenumeration(|| {});

    loop {
        // Wait for USB interrupt
        interrupt::free(|_| {
            unsafe { NVIC::unmask(Interrupt::USB) };
            pwr.sleep_mode(&mut scb).enter();
            NVIC::mask(Interrupt::USB);
            NVIC::unpend(Interrupt::USB);
        });

        if !device.poll(&mut [&mut serial]) {
            continue;
        }

        match echo(&mut serial) {
            Ok(()) | Err(UsbError::WouldBlock) => (),

            Err(error) => {
                led.set_low().unwrap();
                panic!("USB error: {:?}", error);
            }
        }
    }
}


fn echo<Bus>(serial: &mut SerialPort<Bus>)
    -> Result<(), UsbError>
    where Bus: UsbBusTrait
{
    let mut buffer = [0u8; 32];

    let bytes_read = serial.read(&mut buffer)?;

    // Switch all lower-case characters to upper-case
    for b in &mut buffer {
        if *b >= 0x61 && *b <= 0x7a {
            *b -= 0x20;
        }
    }

    let mut offset = 0;
    while offset < bytes_read {
        let bytes_written = serial.write(&buffer[offset .. bytes_read])?;
        offset += bytes_written;
    }

    Ok(())
}

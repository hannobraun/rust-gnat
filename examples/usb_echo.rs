#![no_main]
#![no_std]


extern crate panic_halt;


use cortex_m_rt::entry;
use gnat::hal::{
    prelude::*,
    pac,
    rcc,
    syscfg::SYSCFG,
    usb,
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
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc    = dp.RCC.freeze(rcc::Config::hsi16());
    let mut syscfg = SYSCFG::new(dp.SYSCFG_COMP, &mut rcc);
    let     gpioa  = dp.GPIOA.split(&mut rcc);
    let     gpiob  = dp.GPIOB.split(&mut rcc);

    let mut led = gpiob.pb12.into_push_pull_output();
    led.set_high().unwrap(); // disable LED

    usb::init(&mut rcc, &mut syscfg, dp.CRS);

    let usb_dm = gpioa.pa11;
    let usb_dp = gpioa.pa12;

    let     bus    = UsbBus::new(dp.USB, (usb_dm, usb_dp));
    let mut serial = SerialPort::new(&bus);

    // Use special VID/PID for testing from pid.codes.
    // http://pid.codes/1209/0001/
    let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("MANUFACTURER")
        .product("PRODUCT")
        .serial_number("SERIAL_NUMBER")
        .device_class(USB_CLASS_CDC)
        .build();

    loop {
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

    let mut offset = 0;
    while offset < bytes_read {
        let bytes_written = serial.write(&buffer[offset .. bytes_read])?;
        offset += bytes_written;
    }

    Ok(())
}

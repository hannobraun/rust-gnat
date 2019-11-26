#![no_main]
#![no_std]


extern crate panic_halt;


use core::{
    cell::RefCell,
    fmt::{
        self,
        Write as _,
    },
    str::from_utf8,
};

use cortex_m::{
    interrupt::Mutex,
    singleton,
};
use cortex_m_rt::entry;
use gnat::hal::{
    prelude::*,
    gpio::{
        Output,
        PushPull,
        gpiob::PB12,
    },
    pac::{
        self,
        NVIC,
        Interrupt,
        interrupt,
    },
    pwr::PWR,
    rcc,
    syscfg::SYSCFG,
    timer::Timer,
    usb::{
        USB,
        UsbBusType,
    },
};
use nb::block;
use stm32_usbd::UsbBus;
use usbd_serial::{
    SerialPort,
    USB_CLASS_CDC,
};
use usb_device::{
    UsbError,
    bus::UsbBusAllocator,
    device::{
        UsbDevice,
        UsbDeviceBuilder,
        UsbVidPid,
    },
};


/// The timer that is used to regularly update the USB connection.
static TIMER: Mutex<RefCell<Option<Timer<pac::TIM2>>>> =
    Mutex::new(RefCell::new(None));

/// Writes data to a serial port on the host PC via USB
static WRITER: UsbWriter = UsbWriter::new();

/// Used to signal, if there's a panic.
static mut LED: Option<PB12<Output<PushPull>>> = None;


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

    // Configure the timer run regularly and enable interrupts
    let mut timer = dp.TIM2.timer(5.khz(), &mut rcc);
    timer.listen();

    cortex_m::interrupt::free(|cs| {
        *TIMER.borrow(cs).borrow_mut() = Some(timer);
    });

    let mut led = gpiob.pb12.into_push_pull_output();
    led.set_high().unwrap();

    // Safe, as no interrupt handlers are configured yet. We have exlusive
    // access to all statics.
    unsafe {
        LED = Some(led);
    }

    let usb_dm = gpioa.pa11;
    let usb_dp = gpioa.pa12;

    let hsi48 = rcc.enable_hsi48(&mut syscfg, dp.CRS);
    let usb   = USB::new(dp.USB, usb_dm, usb_dp, hsi48);

    // `UsbWriter` requires the USB bus to have a static lifetime. We're using
    // the `singleton` macro from `cortex-m` here to achieve this.
    let bus =
        singleton!(
            : UsbBusAllocator<UsbBusType> =
                UsbBus::new(usb)
        )
        .unwrap(); // never panics, as `main` is only called once

    // Initialize the USB serial port
    let serial = SerialPort::new(bus);

    // Initialize the USB device
    //
    // Use special VID/PID for testing from pid.codes:
    // http://pid.codes/1209/0001/
    let device = UsbDeviceBuilder::new(bus, UsbVidPid(0x1209, 0x0001))
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

    // Initialize the USB writer by passing it ownership of the USB serial port
    // and device.
    WRITER.init(serial, device);

    // Safe, as there's no critical section here that this could interfere with.
    unsafe { NVIC::unmask(Interrupt::TIM2) };

    let mut i = 0;

    loop {
        // Write data to the USB serial port. This will block until all data has
        // been written into the USB buffer.
        //
        // Please note how this writes more bytes than we have space in the USB
        // writer buffer. You can increase the size of the array to further
        // stress-test the writer's ability to handle large amounts of data.
        write!(&WRITER, "{} {}\r\n", i, from_utf8(&[b'.'; 128]).unwrap())
            .expect("Failed to write to buffer"); // should never happen

        i += 1;

        // Sleep for a bit after writing. Since the timer interrupt will be
        // running regularly, we'll be woken up as soon as the `UsbWriter` had a
        // change to process the data we've written.
        pwr.sleep_mode(&mut scb).enter();
    }
}


/// The interrupt handler that regularly updates the USB connection
///
/// Please note that this is deliberately not a handler for the USB interrupt. I
/// don't know why, but the USB interrupt fires so often, as to drown out any
/// other program activity.
#[interrupt]
fn TIM2() {
    // Reset the timer interrupt. Otherwise it'll immediately refire after we
    // leave this handler.
    cortex_m::interrupt::free(|cs| {
        let mut timer  = TIMER.borrow(cs).borrow_mut();

        if let Some(ref mut timer) = *timer {
            timer.clear_irq();
        }
    });

    // Update the USB writer. Panic, if an error occurs.
    if let Err(error) = WRITER.update() {
        // Safe, as after initialization, we have exlusive access to `LED`.
        if let Some(led) = unsafe { &mut LED } {
            led.set_low().unwrap();
            panic!("USB error: {:?}", error);
        }
    }
}


/// Encapsulates the a USB connection and handles writing to it.
///
/// This struct is designed to live in a static variable, where it can be
/// written to and updated from anywhere. It is also possible to use this struct
/// in a purely single-threaded context, but then its thread safety overhead
/// will be higher than necessary.
///
/// An instance of this struct can be created by calling [`UsbWriter::new`].
///
/// Please note that this struct, although it could theoretically be totally
/// portable, is specific to the STM32L0. This is because making it portable
/// would require a generic argument with trait bounds. Combining generic
/// arguments with trait bounds and const functions (which the constructor needs
/// to be) currently requires Rust nightly.
///
/// Until this limitation is lifted, this struct can't live in any platform-
/// independent crate, but I don't think it would be possible to add it to
/// `stm32l0xx-hal` either. Since `stm32-usbd` depends on `stm32l0xx-hal`, this
/// would result in a circular dependency.
pub struct UsbWriter {
    /// The USB connection
    usb: Mutex<RefCell<Option<Usb>>>,

    /// The write buffer
    buffer: Mutex<RefCell<Buffer>>,
}

impl UsbWriter {
    /// Creates an instance of `UsbWriter`
    ///
    /// The returned instance needs to be initialized before it's possible to
    /// use it, by calling [`UsbWriter::init`].
    pub const fn new() -> Self {
        Self {
            usb:    Mutex::new(RefCell::new(None)),
            buffer: Mutex::new(RefCell::new(Buffer::new())),
        }
    }

    /// Initialize this instance of `UsbWriter`
    pub fn init(&self,
        serial: SerialPort<'static, UsbBusType>,
        device: UsbDevice<'static, UsbBusType>,
    ) {
        cortex_m::interrupt::free(|cs| {
            *self.usb.borrow(cs).borrow_mut() = Some(
                Usb {
                    serial,
                    device,
                }
            )
        });
    }

    /// Update the USB connection
    ///
    /// This method needs to be called regularly, otherwise the USB connection
    /// to the host computer will be lost.
    ///
    /// According to the documentation of `usb-device`, the USB device needs to
    /// be polled every 10 ms to be USB-compliant, but I've found this not to be
    /// enough to keep a connection. Please make sure you call this at a high
    /// enough rate that works for your use case. I've found an update rate of 5
    /// kHz to be sufficient.
    ///
    /// I don't know why this rate needs to be so much higher than stated in the
    /// documentation of `usb-device`. It's possible I'm doing something wrong
    /// or that the update requirement for a serial connection is way higher
    /// than the theoretical minimum.
    ///
    /// While it's possible to call this function from the main program, this
    /// would be a very fragile solution, especially if enough data is written
    /// to risk filling up the buffer. I recommend calling this function from a
    /// timer interrupt.
    pub fn update(&self) -> Result<(), UsbError> {
        cortex_m::interrupt::free(|cs| {
            let mut usb    = self.usb.borrow(cs).borrow_mut();
            let mut buffer = self.buffer.borrow(cs).borrow_mut();

            if let Some(ref mut usb) = *usb {
                if !usb.device.poll(&mut [&mut usb.serial]) {
                    return Ok(());
                }

                match buffer.read(|data| usb.serial.write(data)) {
                    Ok(_) | Err(UsbError::WouldBlock) => return Ok(()),
                    Err(error) =>                        return Err(error),
                }
            }

            Ok(())
        })
    }

    /// Writes data to the USB serial connection
    ///
    /// This method will either write all data to the buffer in one go, or
    /// return `Err(nb::Error::WouldBlock)`.
    ///
    /// Calling this function will, by itself, do nothing useful. You must call
    /// [`UsbWriter::update`] regularly, to make sure the buffered data gets
    /// written to the connection.
    ///
    /// The data passed to this method must fit into the buffer, otherwise
    /// `Err(nb::Other(BufferTooSmallError))` is returned. If you need to write
    /// more data and are fine with blocking, consider calling
    /// [`UsbWriter::write_blocking`] instead.
    ///
    /// This method is thread-safe, meaning there is no synchronization required
    /// to use it from multiple threads.
    pub fn write(&self, data: &[u8]) -> nb::Result<(), BufferTooSmallError> {
        cortex_m::interrupt::free(|cs| {
            let mut buffer = self.buffer.borrow(cs).borrow_mut();

            if data.len() > buffer.data.len() {
                return Err(nb::Error::Other(BufferTooSmallError));
            }

            let free_space = buffer.data.len() - buffer.last;
            if data.len() > free_space {
                return Err(nb::Error::WouldBlock);
            }

            let start = buffer.last;
            let end   = buffer.last + data.len();

            buffer.data[start .. end].copy_from_slice(data);
            buffer.last += data.len();

            Ok(())
        })
    }

    /// Blockingly write data to the USB serial connection
    ///
    /// Writes data to the serial connection, blocking until all data has been
    /// written into the buffer. Please note that this method is not optimized
    /// for high-throughput situations. If a higher-priority thread keeps
    /// writing data to the buffer, this method could block forever.
    ///
    /// Calling this function will, by itself, do nothing useful. You must call
    /// [`UsbWriter::update`] regularly, to make sure the buffered data gets
    /// written to the connection.
    ///
    /// This method is thread-safe, meaning there is no synchronization required
    /// to use it from multiple threads.
    pub fn write_blocking(&self, data: &[u8]) {
        let buffer_size = cortex_m::interrupt::free(|cs| {
            let buffer = self.buffer.borrow(cs).borrow();
            buffer.data.len()
        });

        let mut offset = 0;

        while offset < data.len() {
            let bytes_to_write = usize::min(data.len(), buffer_size);

            block!(self.write(&data[offset..bytes_to_write]))
                // Can't panic, as we made sure not to write more bytes than can
                // fit the buffer.
                .unwrap();

            offset += bytes_to_write;
        }
    }
}

impl fmt::Write for &'_ UsbWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write_blocking(s.as_bytes());
        Ok(())
    }
}


#[derive(Debug)]
pub struct BufferTooSmallError;


/// Encapsulates the USB connection
struct Usb {
    /// The USB serial port
    serial: SerialPort<'static, UsbBusType>,

    /// The USB device
    device: UsbDevice<'static, UsbBusType>,
}


/// The write buffer of `UsbWriter`
///
/// This is a straight-forward implementation of a write buffer. It is almost
/// but not quite a circular buffer. It acts like a circular buffer until the
/// end of the buffer is reached, at which point no more data can be written.
/// When the buffer has been emptied, it is reset, and data can be written
/// again.
///
/// While this buffer wasn't written for performance, I actually expect this to
/// be a quite performant approach, as long as data can be written to the USB
/// connection at a higher rate than data is written into the buffer. If the
/// amount of data written to the buffer outstrips the rate at which it is being
/// emptied, then writers will be blocked more often and longer than strictly
/// required.
///
/// In any case, I expect the overhead of synchronizing access to the buffer to
/// be much larger than the performance cost of the buffer itself, although I
/// didn't measure this.
///
/// If more performance is required, it might make most sense to switch to a
/// generic solution, like `heapless::spsc` or `bbqueue`. However, the interface
/// of `heapless::spsc` isn't well-suited to the needs of `UsbWriter`, and, as
/// of this writing, `bbqueue` doesn't support ARMv6-M (i.e. Cortex-M0/M0+).
struct Buffer {
    /// The buffer itself
    ///
    /// In theory, we could use the `generic-array` crate to make this buffer
    /// generic over its length. In practice, we run into some limitations of
    /// `const fn` when trying to do that, namely that trait bounds on `const
    /// fn` require nightly.
    ///
    /// For now, we just have to choose an appropriate size for our target
    /// platform, but long-term this shouldn't be a problem anymore, either
    /// because of improvements to `const fn` make `GenericArray` usable, or
    /// because const generics become available and make `GenericArray
    /// unnecessary.
    data: [u8; 64],

    /// Points to the start of valid data in the buffer
    first: usize,

    /// Points to one byte after the end of valid data in the buffer
    last: usize,
}

impl Buffer {
    /// Create an empty buffer
    const fn new() -> Self {
        Self {
            data:  [0; 64],
            first: 0,
            last:  0,
        }
    }

    /// Read data from the buffer
    ///
    /// Calls the provided closure, passing it a slice over all valid data in
    /// the buffer. Expects the closure to return how many bytes were taken from
    /// the buffer, and removes that many bytes after the closure returns.
    fn read<F, Error>(&mut self, f: F) -> Result<usize, Error>
        where F: FnOnce(&[u8]) -> Result<usize, Error>
    {
        // Call closure, passing it all valid data in the buffer.
        let result = f(&self.data[self.first..self.last]);

        // If the closure returned an error, we don't need to know and just pass
        // it on below. If it was successful, we need to remove the bytes it
        // read from the buffer.
        if let Ok(bytes_read) = result {
            // Remove any bytes read by the closure from the buffer. This should
            // be correct, even if the closure returns crap:
            // - First, we're using a saturing addition, so the index can't
            //   overflow.
            // - Second, even if the closure returns a larger number than we had
            //   bytes in the buffer, we catch that below by not relying on what
            //   usually would be true, that `first` is not larger than `last`.
            self.first = self.first.saturating_add(bytes_read);

            // If the buffer is empty, reset it.
            if self.first >= self.last {
                self.first = 0;
                self.last  = 0;
            }
        }

        result
    }
}

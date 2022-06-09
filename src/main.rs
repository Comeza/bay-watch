//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_time::rate::*;
use hal::gpio::{FunctionSpi, Output, PushPull, Spi};
use hal::spi::{Enabled, SpiDevice};
use mfrc522::Mfrc522;
use panic_halt as _;
use rp2040_hal::spi::Spi;
use rp_pico::entry;
use rp_pico::hal::{self, pac, prelude::*};
use usbd_serial::SerialPort;

// USB Device support
use core::fmt::Write;
use usb_device::{class_prelude::*, prelude::*};

fn cards<'a>() -> [&'a [u8]; 11] {
    let cards = [
        [35, 188, 68, 167],
        [243, 172, 67, 167],
        [147, 204, 155, 167],
        [227, 164, 122, 167],
    ];
    let transponders = [
        [64, 107, 20, 27],
        [144, 109, 125, 34],
        [18, 95, 211, 27],
        [97, 51, 50, 39],
    ];
    let aaron = [4, 29, 61, 74, 27, 92, 128];
    let valle = [4, 27, 16, 74, 83, 102, 128];
    let dennis = [4, 49, 21, 178, 51, 92, 128];
    [
        &[35, 188, 68, 167],
        &[243, 172, 67, 167],
        &[147, 204, 155, 167],
        &[227, 164, 122, 167],
        &[64, 107, 20, 27],
        &[144, 109, 125, 34],
        &[18, 95, 211, 27],
        &[97, 51, 50, 39],
        &[4, 29, 61, 74, 27, 92, 128],
        &[4, 27, 16, 74, 83, 102, 128],
        &[4, 49, 21, 178, 51, 92, 128],
    ]
}
fn check_rfid<D: SpiDevice>(mfrc: Mfrc522<Spi<Enabled, D, 8>, Output<PushPull>>) -> bool {
    if let Ok(atqa) = mfrc.reqa() {
        if let Ok(uid) = mfrc.select(&atqa) {
            let mut buf = [0u8; 64];
            /*let _ = match write_to::show(&mut buf, format_args!("uid: {:?}\r\n", &uid.as_bytes())) {
                Ok(uid) => serial.write(uid.as_bytes()),
                Err(_) => serial.write(b"uid to long for buffer\r\n"),
            };*/
            return cards().iter().any(|id| id == &uid.as_bytes());
        }
    }
    false
}

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    use embedded_hal::spi::MODE_0;
    use embedded_time::rate::*;
    use rp2040_hal::{
        gpio::{FunctionSpi, Pins},
        pac,
        spi::Spi,
    };

    let _ = pins.gpio2.into_mode::<FunctionSpi>();
    let _ = pins.gpio3.into_mode::<FunctionSpi>();
    let _ = pins.gpio4.into_mode::<FunctionSpi>();

    let spi = Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1_000_000u32.Hz(),
        &MODE_0,
    );

    /*let spi = Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );*/

    let nss_pin = pins.gpio1.into_push_pull_output();
    let mut mfrc = Mfrc522::new(spi, nss_pin).unwrap();

    let mut summer_pin = pins.gpio22.into_push_pull_output();
    let mut door = pins.gpio28.into_pull_up_input();
    summer_pin.set_low();

    let mut state = State::Armed;
    const SUMMER_INTERVAL: u32 = 500;
    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);
    // Set the LED to be an output
    let mut led = pins.led.into_push_pull_output();

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    //watchdog.start(1_050_000.microseconds());

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;
    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 10_000 {
            said_hello = true;
            let _ = serial.write(b"Test 6AcAb?\r\n");
        }

        //let _ = serial.write(b"no mfrc connected\r\n");
        let _ = serial.flush();

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_lowercase();
                    });

                    if buf.contains(&b'1') {
                        led.set_high().unwrap();
                    } else if buf.contains(&b'0') {
                        led.set_low().unwrap();
                    }
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
        mfrc.hlta();
        mfrc.stop_crypto1();
    }
}

enum State {
    Armed,
    Snoozed(u32),
    Alarming(u32, bool),
    Disarmed,
}

pub mod write_to {
    use core::cmp::min;
    use core::fmt;

    pub struct WriteTo<'a> {
        buffer: &'a mut [u8],
        // on write error (i.e. not enough space in buffer) this grows beyond
        // `buffer.len()`.
        used: usize,
    }

    impl<'a> WriteTo<'a> {
        pub fn new(buffer: &'a mut [u8]) -> Self {
            WriteTo { buffer, used: 0 }
        }

        pub fn as_str(self) -> Option<&'a str> {
            if self.used <= self.buffer.len() {
                // only successful concats of str - must be a valid str.
                use core::str::from_utf8_unchecked;
                Some(unsafe { from_utf8_unchecked(&self.buffer[..self.used]) })
            } else {
                None
            }
        }
    }

    impl<'a> fmt::Write for WriteTo<'a> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            if self.used > self.buffer.len() {
                return Err(fmt::Error);
            }
            let remaining_buf = &mut self.buffer[self.used..];
            let raw_s = s.as_bytes();
            let write_num = min(raw_s.len(), remaining_buf.len());
            remaining_buf[..write_num].copy_from_slice(&raw_s[..write_num]);
            self.used += raw_s.len();
            if write_num < raw_s.len() {
                Err(fmt::Error)
            } else {
                Ok(())
            }
        }
    }

    pub fn show<'a>(buffer: &'a mut [u8], args: fmt::Arguments) -> Result<&'a str, fmt::Error> {
        let mut w = WriteTo::new(buffer);
        fmt::write(&mut w, args)?;
        w.as_str().ok_or(fmt::Error)
    }
}

pub fn test() {
    let mut buf = [0u8; 64];
    let _s: &str = write_to::show(
        &mut buf,
        format_args!("write some stuff {:?}: {}", "foo", 42),
    )
    .unwrap();
}

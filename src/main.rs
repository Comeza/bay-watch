//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use embedded_hal::digital::v2::InputPin;
use embedded_hal::{digital::v2::OutputPin, spi::MODE_0};
use embedded_time::rate::*;
use hal::spi::{Enabled, SpiDevice};
use hal::Timer;
use mfrc522::Mfrc522;
use panic_halt as _;
use rp2040_hal::{gpio::FunctionSpi, pac, spi::Spi};
use rp_pico::entry;
use rp_pico::hal::{self, prelude::*};
use usbd_serial::SerialPort;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

fn cards<'a>() -> [&'a [u8]; 11] {
    [
        &[35, 188, 68, 167],            // A
        &[243, 172, 67, 167],           // B
        &[147, 204, 155, 167],          // C
        &[227, 164, 122, 167],          // D
        &[64, 107, 20, 27],             // A
        &[144, 109, 125, 34],           // B
        &[18, 95, 211, 27],             // C
        &[97, 51, 50, 39],              // D
        &[4, 29, 61, 74, 27, 92, 128],  // Aaron
        &[4, 27, 16, 74, 83, 102, 128], // Valle
        &[4, 49, 21, 178, 51, 92, 128], // Dennis
    ]
}
fn check_rfid<D: SpiDevice, NSS: OutputPin>(mfrc: &mut Mfrc522<Spi<Enabled, D, 8>, NSS>) -> bool {
    if let Ok(atqa) = mfrc.reqa() {
        if let Ok(uid) = mfrc.select(&atqa) {
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

    let _ = pins.gpio2.into_mode::<FunctionSpi>();
    let _ = pins.gpio3.into_mode::<FunctionSpi>();
    let _ = pins.gpio4.into_mode::<FunctionSpi>();

    let spi = Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1_000_000u32.Hz(),
        &MODE_0,
    );

    let nss_pin = pins.gpio1.into_push_pull_output();
    let mut mfrc = Mfrc522::new(spi, nss_pin).unwrap();

    let mut summer_pin = pins.gpio22.into_push_pull_output();
    let door = pins.gpio28.into_pull_up_input();

    let mut state = State::Armed;
    const SNOOZE_INTERVAL: u64 = 1_000_000 * 10; //10 seconds
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
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let mut said_hello = false;
    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 10_000 {
            said_hello = true;
            let _ = serial.write(b"Hello 1, World!\r\n");
        }

        //let auth = check_rfid(&mut mfrc);
        let door_is_open = door.is_high().unwrap();
        match door_is_open {
            true => serial.write(b"o"),
            false => serial.write(b"c"),
        };
        /*state = match state {
            State::Armed => match (auth, door_is_open) {
                (true, _) => State::Snoozed(timer.get_counter()),
                (false, false) => State::Armed,
                (false, true) => State::Alarming,
            },
            State::Snoozed(timestamp) => {
                if timer.get_counter() - timestamp > SNOOZE_INTERVAL {
                    //blink
                    serial.write(b"blink").unwrap();
                    State::Armed
                } else if door_is_open {
                    State::Disarmed
                } else if auth {
                    //blink
                    serial.write(b"blink").unwrap();
                    State::Snoozed(timer.get_counter())
                } else {
                    State::Snoozed(timestamp)
                }
            }
            State::Disarmed => {
                if !door_is_open {
                    //blink
                    serial.write(b"blink").unwrap();
                    State::Snoozed(timer.get_counter())
                } else {
                    State::Disarmed
                }
            }
            State::Alarming => {
                if auth {
                    summer_pin.set_low().unwrap();
                    State::Disarmed
                } else {
                    //handle alarm here
                    summer_pin.set_high().unwrap();
                    //blink
                    serial.write(b"blink").unwrap();
                    State::Alarming
                }
            }
        };*/
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
                        b.make_ascii_uppercase();
                    });

                    if buf.contains(&('1' as u8)) {
                        led.set_high().unwrap();
                    } else if buf.contains(&('0' as u8)) {
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
        //mfrc.hlta();
    }
}

enum State {
    Armed,
    Snoozed(u64),
    Alarming,
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

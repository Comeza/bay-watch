//! # Pico Blinky Example
//!
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
use mfrc522::Mfrc522;
use panic_halt as _;
use rp2040_hal::spi::Spi;
use rp_pico::entry;
use rp_pico::hal::{self, pac, prelude::*};
use usbd_serial::SerialPort;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
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
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let spi = Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        1_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );
    let mut mfrc = Mfrc522::new(spi, pins.gpio1.into_push_pull_output()).unwrap();

    let mut summer_pin = pins.gpio22.into_push_pull_output();
    let mut led = pins.led.into_push_pull_output();
    let mut door = pins.gpio28.into_pull_up_input();

    let mut state = State::Armed;
    const SUMMER_INTERVAL: u32 = 500;

    loop {
        if let Ok(atqa) = mfrc.reqa() {
            if let Ok(uid) = mfrc.select(&atqa) {
                let uid_str = uid.as_bytes();
                serial.write(uid_str).ok();
            }
        }

        state = match state {
            State::Armed => {
                // TODO: Check RFID

                if door.is_low().unwrap() {
                    State::Alarming(0, false)
                } else {
                    state
                }
            }
            State::Snoozed(timer) => {
                // TODO: Check RFID

                if timer == 0 {
                    State::Armed
                } else {
                    if door.is_low().unwrap() {
                        State::Disarmed
                    } else {
                        State::Snoozed(timer - 1)
                    }
                }
            }
            State::Alarming(timer, hl) => {
                // TODO: Check RFID
                if timer == 0 {
                    if !hl {
                        summer_pin.set_high();
                    } else {
                        summer_pin.set_low();
                    };
                    State::Alarming(SUMMER_INTERVAL, !hl)
                } else {
                    State::Alarming(timer - 1, hl)
                }
            }
            State::Disarmed => {
                if door.is_high().unwrap() {
                    State::Armed
                } else {
                    State::Disarmed
                }
            }
        };
    }
}

enum State {
    Armed,
    Snoozed(u32),
    Alarming(u32, bool),
    Disarmed,
}

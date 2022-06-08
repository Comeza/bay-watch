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

use cortex_m::prelude::*;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Extensions;
use embedded_time::rate::*;
use embedded_time::Instant;
use hal::clocks;
use hal::spi::SpiDevice;
use hal::timer::CountDown;
use mfrc522::Mfrc522;
use panic_halt as _;
use rp_pico::entry;
use rp2040_hal::spi::Spi;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;

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

    let spi = Spi::<_,_,8>::new(pac.SPI0).init(&mut pac.RESETS, clocks.peripheral_clock.freq(), 1_000_000u32.Hz(), &embedded_hal::spi::MODE_0);
    let mfrc = Mfrc522::new(spi, pins.gpio1.into_push_pull_output()).unwrap();


    if let Ok(atqa) = mfrc.reqa() {
        if let Ok(uid) = mfrc.select(&atqa) {
            
            // TODO
        }
    }


    let mut summer_pin = pins.gpio22.into_push_pull_output();
    let mut led = pins.led.into_push_pull_output();
    let mut door = pins.gpio28.into_pull_up_input();
    let timer = 0u32;
    let mut state = State::Armed;
    const SUMMER_INTERVAL: u32 = 500;

    loop {
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
            },
        };
    }
}

enum State {
    Armed,
    Snoozed(u32),
    Alarming(u32, bool),
    Disarmed,
}

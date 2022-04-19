#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

pub mod controller;

use arduino_hal::I2c;
use controller::inputs::{millis::millis_init, PositionInput};
use panic_halt as _;
use ufmt::uwriteln;

#[arduino_hal::entry]
fn main() -> ! {
    // Retrieve resources
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    millis_init(dp.TC0);

    let mut pos_input = PositionInput::new(I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    ));

    pos_input.init().unwrap();
    pos_input.calibrate().unwrap();

    // Begin program loop
    loop {
        pos_input.read().unwrap();
        uwriteln!();
        arduino_hal::delay_ms(10);
    }
}

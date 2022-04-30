#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

pub mod controller;

use arduino_hal::I2c;
use arduino_millis_driver::*;
use arduino_utils::*;
use controller::model::*;
use elinalgebra::*;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    // Retrieve resources
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut delay = arduino_hal::Delay::new();

    millis_init(dp.TC0).unwrap();

    let mut pos_input = MpuPositionInput::new(I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    ));
    pos_input.init(&mut delay).unwrap();

    pos_input
        .calibrate()
        .unwrap_or_else(|_| upanic!(&mut serial, "Mpu calibration failed"));

    let mut pos_data = PositionData::new();

    uprint!(&mut serial, "Finished setup");
    // Begin program loop
    loop {
        pos_input
            .read(&mut pos_data)
            .unwrap_or_else(|_| upanic!(&mut serial, "Positional read failed"));

        uprint!(
            &mut serial,
            "acc: {}, {}, {} ",
            pos_data.gyro_angle.x as i32,
            pos_data.gyro_angle.y as i32,
            pos_data.gyro_angle.z as i32,
        );
    }
}

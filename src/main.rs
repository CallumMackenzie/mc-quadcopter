#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

pub mod controller;

use arduino_hal::I2c;
use arduino_millis_driver::millis_init;
use arduino_utils::{upanic, uprint};
use controller::model::*;
use mpu6050_driver::{AccelRange, GyroRange};
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    // Retrieve resources
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut delay = arduino_hal::Delay::new();

    millis_init(dp.TC0)
        .unwrap_or_else(|e| upanic!(&mut serial, "Could not initialize millis: {}", e as u8));

    let mut pos_input = MpuPositionInput::new(I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    ));
    pos_input
        .init(&mut delay)
        .unwrap_or_else(|_| upanic!(&mut serial, "Mpu init failed"));

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
        let a = pos_data.gyro_acc;
        uprint!(
            &mut serial,
            "{}, {}, {}",
            a.x as i32,
            a.y as i32,
            a.z as i32,
        );

        arduino_hal::delay_ms(5);
    }
}

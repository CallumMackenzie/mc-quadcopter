#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

pub mod controller;

use arduino_hal::I2c;
use arduino_millis_driver::{millis_init, millis};
use mpu6050_driver::{AccelRange, GyroRange};
use controller::inputs::PositionInput;
use panic_halt as _;
use ufmt::uwriteln;

#[arduino_hal::entry]
fn main() -> ! {
    // Retrieve resources
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut delay = arduino_hal::Delay::new();
	uwriteln!(&mut serial, "Retrieved peripherals").unwrap();

    millis_init(dp.TC0).unwrap_or_else(|e| {
		uwriteln!(&mut serial, "Could not initialize millis: {}", e as u8).unwrap();
		loop{}
	});
	uwriteln!(&mut serial, "Initialized millis").unwrap();

    let mut pos_input = PositionInput::new(I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    ));
	uwriteln!(&mut serial, "Constructed MPU").unwrap();
    pos_input.init(&mut delay).unwrap_or_else(|_| {
		uwriteln!(&mut serial, "Mpu init failed").unwrap();
		loop{}
	});
	uwriteln!(&mut serial, "Initialized mpu").unwrap();
	pos_input.mpu.set_accel_range(AccelRange::G8);
	pos_input.mpu.set_gyro_range(GyroRange::D1000);
    pos_input.calibrate().unwrap_or_else(|_| {
		uwriteln!(&mut serial, "Mpu calibration failed").unwrap();
		loop{}
	});

	uwriteln!(&mut serial, "Finished setup").unwrap();

	let mut lf = millis();

    // Begin program loop
    loop {
        // pos_input.read().unwrap();
		pos_input.read().unwrap_or_else(|_| {
			uwriteln!(&mut serial, "Gyro read failed").unwrap();
			loop{}
		});
		let a = pos_input.gyro_angle;
        uwriteln!(
            &mut serial,
            "{}, {}, {}, diff: {} ms",
            a.x as i32,
            a.y as i32,
            a.z as i32,
			millis() - lf,
        ).unwrap();
		lf = millis();
    }
}

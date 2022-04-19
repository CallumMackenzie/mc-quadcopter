#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

pub mod controller;
pub mod millis;

use controller::mpu6050::Mpu6050;
use millis::millis_init;
use panic_halt as _;
use ufmt::uwriteln;

#[arduino_hal::entry]
fn main() -> ! {
    // Retrieve resources
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    millis_init(dp.TC0);
    unsafe { avr_device::interrupt::enable() };

    // Set up i2c bus driver
    let mut i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );

    // Set up systems
    let mut mpu = Mpu6050::new(&mut i2c);
    mpu.init().unwrap_or_else(|_| {
        uwriteln!(&mut serial, "Mpu6050 initialization error").unwrap();
        loop {}
    });
    mpu.calculate_all_imu_error(100).unwrap();

    // Begin program loop
    loop {
        let gyro = mpu.read_acc().unwrap();
        uwriteln!(
            &mut serial,
            "x: {}, y: {}, z: {}",
            (gyro.x * 9.81) as i32,
            (gyro.y * 9.81) as i32,
            (gyro.z * 9.81) as i32
        )
        .unwrap();
        arduino_hal::delay_ms(10);
    }
}

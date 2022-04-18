#![no_std]
#![no_main]

pub mod mpu6050;

use mpu6050::Mpu6050;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );

    let mut mpu = Mpu6050::new(&mut i2c);
    mpu.init().unwrap();

    loop {
        arduino_hal::delay_ms(1000);
    }
}

struct ControllerInput {}

impl ControllerInput {}

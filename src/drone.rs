use alloc::boxed::Box;

use cortex_m::delay::Delay;

use defmt_rtt as _;

use adafruit1893_driver::Adafruit1893;
use fugit::RateExtU32;
use motor_driver::{Motor, MotorManager};
use mpu6050_driver::Mpu6050;
use panic_halt as _;
use rp2040_hal::gpio::bank0::{Gpio0, Gpio1, Gpio14, Gpio15, Gpio2, Gpio3, Gpio8, Gpio9};
use rp2040_hal::gpio::{FunctionI2C, Pin, PinId, PullDownDisabled, PushPullOutput};
use rp2040_hal::pac::I2C0;
use rp2040_hal::pwm::{FreeRunning, Pwm0, Pwm1, Slice};
use rp2040_hal::{clocks::SystemClock, I2C};
use rp_pico::hal::prelude::*;
use rp_pico::pac::{I2C1, RESETS};

pub fn setup_motors<LED: PinId>(
    delay: &mut Delay,
    led: &mut Pin<LED, PushPullOutput>,
    mut pwm0: Slice<Pwm0, FreeRunning>,
    mut pwm1: Slice<Pwm1, FreeRunning>,
    p0: Pin<Gpio0, PullDownDisabled>,
    p1: Pin<Gpio1, PullDownDisabled>,
    p2: Pin<Gpio2, PullDownDisabled>,
    p3: Pin<Gpio3, PullDownDisabled>,
) -> MotorManager {
    // Configure PWM
    pwm0.set_ph_correct();
    pwm0.set_div_int(20u8); // 50 hz
    pwm0.enable();
    pwm1.set_ph_correct();
    pwm1.set_div_int(20u8); // 50 hz
    pwm1.enable();

    let motor0 = Box::new(Motor::new_a(pwm0.channel_a, 20, p0));
    let motor1 = Box::new(Motor::new_b(pwm0.channel_b, 20, p1));
    let motor2 = Box::new(Motor::new_a(pwm1.channel_a, 20, p2));
    let motor3 = Box::new(Motor::new_b(pwm1.channel_b, 20, p3));
    let mut motor_manager = MotorManager::new([motor0, motor1, motor2, motor3]);
    motor_manager.setup(led, delay).unwrap();
    return motor_manager;
}

pub fn setup_mpu6050(
    i2c1: I2C1,
    gpio14: Pin<Gpio14, PullDownDisabled>,
    gpio15: Pin<Gpio15, PullDownDisabled>,
    resets: &mut RESETS,
    system_clock: &SystemClock,
    delay: &mut Delay,
) -> Mpu6050<I2C<I2C1, (Pin<Gpio14, FunctionI2C>, Pin<Gpio15, FunctionI2C>)>> {
    let mut mpu = Mpu6050::new(I2C::i2c1(
        i2c1,
        gpio14.into_mode(),
        gpio15.into_mode(),
        400.kHz(),
        resets,
        system_clock.freq().to_Hz().Hz(),
    ));
    mpu.init(delay).unwrap();
    mpu.calculate_all_imu_error(10).unwrap();
    mpu
}

pub fn setup_adafruit1893(
    i2c0: I2C0,
    gpio8: Pin<Gpio8, PullDownDisabled>,
    gpio9: Pin<Gpio9, PullDownDisabled>,
    resets: &mut RESETS,
    system_clock: &SystemClock,
) -> Adafruit1893<I2C<I2C0, (Pin<Gpio8, FunctionI2C>, Pin<Gpio9, FunctionI2C>)>> {
    let a1893 = Adafruit1893::new(I2C::i2c0(
        i2c0,
        gpio8.into_mode(),
        gpio9.into_mode(),
        400.kHz(),
        resets,
        system_clock.freq().to_Hz().Hz(),
    ));
    a1893
}

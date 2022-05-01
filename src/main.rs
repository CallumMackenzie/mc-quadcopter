#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

pub mod controller;

use arduino_utils::*;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    // mpu_test();
    servo_test();
}

fn mpu_test() -> ! {
    use arduino_hal::I2c;
    use arduino_millis_driver::*;
    use controller::model::*;

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

    pos_input.calibrate().unwrap();

    let mut pos_data = PositionData::new();

    uprint!(&mut serial, "Finished setup");
    // Begin program loop
    loop {
        pos_input.read(&mut pos_data).unwrap();

        uprint!(
            &mut serial,
            "acc: {}, {}, {} ",
            pos_data.gyro_angle.x as i32,
            pos_data.gyro_angle.y as i32,
            pos_data.gyro_angle.z as i32,
        );
    }
}

fn servo_test() -> ! {
	use servo_driver::{Traxxas2080Servo, ServoDriver};
	use core::cell::RefCell;

    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

	let tc2_rc = RefCell::new(dp.TC2);
	let mut servo_1 = Traxxas2080Servo::pd3(&tc2_rc, pins.d3.into_output());
	let mut servo_2 = Traxxas2080Servo::pb3(&tc2_rc, pins.d11.into_output());

    loop {
		for angle in [0f32, 45f32, -45f32].iter() {
			servo_1.set_angle(*angle);
			servo_2.set_angle(-*angle);
			uprint!(&mut serial, "Angle: {}", *angle as i32);
			arduino_hal::delay_ms(1000);
		}
    }
}

fn radio_test() -> ! {
    use arduino_hal::spi;
    use nrf24_rs::{
        config::{NrfConfig, PALevel},
        Nrf24l01, SPI_MODE,
    };

    // Retrieve resources
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut delay = arduino_hal::Delay::new();

    let ce = pins.d7.into_output(); // Chip enable
    let ss = pins.d10.into_output(); // Exchange ss for cs
    let sck = pins.d13.into_output(); // Serial clock
    let mosi = pins.d11.into_output();
    let miso = pins.d12.into_pull_up_input();

    let settings = spi::Settings {
        data_order: spi::DataOrder::MostSignificantFirst,
        clock: spi::SerialClockRate::OscfOver4,
        mode: SPI_MODE,
    };
    let (spi, ncs) = spi::Spi::new(dp.SPI, sck, mosi, miso, ss, settings);

    let message = b"Test!";

    let config = NrfConfig::default()
        .channel(8)
        .pa_level(PALevel::Min)
        .payload_size(message.len() as u8);

    let mut nrf_chip = Nrf24l01::new(spi, ce, ncs, &mut delay, config).unwrap();
    if !nrf_chip.is_connected().unwrap() {
        upanic!(&mut serial, "Could not connect");
    }
    nrf_chip.open_writing_pipe(b"drone").unwrap();

    while let Err(_) = nrf_chip.write(&mut delay, message) {
        arduino_hal::delay_ms(50);
    }

    loop {}
}

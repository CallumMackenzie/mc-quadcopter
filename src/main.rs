#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use alloc::string::{String, ToString};
use core::alloc::{GlobalAlloc, Layout};

use cortex_m::asm::delay;
use cortex_m::delay::Delay;
use defmt::export::str;
use defmt_rtt as _;
use embedded_alloc::Heap;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_hal::timer::{Cancel, CountDown};
use fugit::{ExtU32, RateExtU32};
use mpu6050_driver::Mpu6050;
use panic_halt as _;
use rp2040_hal::{clocks::SystemClock, I2C, Timer};
use rp2040_hal::gpio::{FunctionI2C, Pin, PullDownDisabled, PushPullOutput, ValidPinMode};
use rp2040_hal::gpio::bank0::{BankPinId, Gpio0, Gpio14, Gpio15};
use rp2040_hal::pwm::{ChannelId, Slices, ValidPwmOutputPin};
use rp_pico::{hal, Pins};
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
use rp_pico::pac::{I2C1, RESETS};
use ufmt::uwriteln;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use motor_driver::{Motor, MotorManager};

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rp2040_hal::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    // The default is to generate a 125 MHz system clock
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

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));


    let mut serial = SerialPort::new(&usb_bus);
    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Callum Mackenzie")
        .product("Raspberry Pi Pico Drone")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC) // from: https://www.usb.org/defined-class-codes
        .build();

    // Init PWMs
    let mut pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    let mut led = pins.led.into_push_pull_output();
    // let mut motor_manager = setup_motors(&mut delay, &mut pwm_slices);
    let mut mpu6050 = setup_accelerometer(&clocks.system_clock,
                                          &mut pac.RESETS,
                                          &mut delay,
                                          pac.I2C1,
                                          pins.gpio14,
                                          pins.gpio15);

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut greeted = false;

    loop {
        if !greeted && timer.get_counter().ticks() > 2_000_000 {
            serial.write("Welcome to Drone USB Interface\r\n".as_bytes()).unwrap();
            greeted = true;
        }

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(count) => {
                    buf.iter_mut().take(count)
                        .for_each(|x| match *x as char {
                            't' => {
                                let tmp = mpu6050.read_temp().unwrap();
                                let str = format!("MPU6050 Temp: {:.2}\r\n", tmp);
                                serial.write(str.as_bytes()).unwrap();
                            }
                            'a' => {
                                let acc = mpu6050.read_acc().unwrap();
                                let str =
                                    format!("Planar acceleration: {:.2}, {:.2}, {:.2}\r\n",
                                            acc.x, acc.y, acc.z);
                                serial.write(str.as_bytes()).unwrap();
                            }
                            'g' => {
                                let gyro = mpu6050.read_gyro().unwrap();
                                let str =
                                    format!("Gyro acceleration: {:.2}, {:.2}, {:.2}\r\n",
                                            gyro.x, gyro.y, gyro.z);
                                serial.write(str.as_bytes()).unwrap();
                            }
                            _ => {}
                        });
                }
            }
        }
    }
}

// fn setup_motors<'a, LED: PinId>(
//     delay: &'a mut Delay,
//     pwm_slices: &'a mut Slices,
//     led: &mut Pin<LED, PushPullOutput>,
//     pins: Pins,
// ) -> MotorManager<'a> {
// // Configure PWM0
//     let pwm = &mut pwm_slices.pwm0;
//     pwm.set_ph_correct();
//     pwm.set_div_int(20u8); // 50 hz
//     pwm.enable();
//
// // Output channel B on PWM0 to the GPIO1 pin
//     let mut motor1 = Motor::new_b(&mut pwm.channel_b, 20, pins.gpio1);
//
// // This needs to be done ASAP
//     let mut motor_manager = MotorManager {
//         motors: &mut [&mut motor1],
//     };
//
//     motor_manager.setup(led, delay).unwrap();
//     return motor_manager;
// }

fn setup_accelerometer(system_clock: &SystemClock,
                       resets: &mut RESETS,
                       delay: &mut Delay,
                       i2c1: I2C1,
                       gpio14: Pin<Gpio14, PullDownDisabled>,
                       gpio15: Pin<Gpio15, PullDownDisabled>)
                       -> Mpu6050<I2C<I2C1, (Pin<Gpio14, FunctionI2C>, Pin<Gpio15, FunctionI2C>)>> {
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
    return mpu;
}

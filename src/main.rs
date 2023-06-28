#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
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
use rp2040_hal::gpio::{FunctionI2C, Pin, PinId, PullDownDisabled, PushPullOutput, ValidPinMode};
use rp2040_hal::gpio::bank0::{BankPinId, Gpio0, Gpio1, Gpio14, Gpio15, Gpio2, Gpio3, Gpio8, Gpio9};
use rp2040_hal::pac::I2C0;
use rp2040_hal::pwm::{ChannelId, FreeRunning, Pwm0, Pwm1, Slice, Slices, ValidPwmOutputPin};
use rp_pico::{hal, Pins};
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
use rp_pico::pac::{I2C1, RESETS};
use ufmt::uwriteln;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use adafruit1893_driver::{Adafruit1893, Adafruit1893Error};
use motor_driver::{Motor, MotorManager};

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[rp2040_hal::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024 * 2;
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
    let mut motor_manager =
        setup_motors(&mut delay,
                     &mut led,
                     pwm_slices.pwm0,
                     pwm_slices.pwm1,
                     pins.gpio0,
                     pins.gpio1,
                     pins.gpio2,
                     pins.gpio3,
        );
    let mut mpu6050 = setup_mpu6050(
        pac.I2C1,
        pins.gpio14,
        pins.gpio15,
        &mut pac.RESETS,
        &clocks.system_clock,
        &mut delay,
    );
    let mut a1893 = setup_adafruit1893(
        pac.I2C0,
        pins.gpio8,
        pins.gpio9,
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(count) => {
                    buf.iter_mut().take(count).for_each(|x| match *x as char {
                        't' => {
                            let tmp = mpu6050.read_temp().unwrap();
                            let str = format!("MPU6050 Temp: {:.2}\r\n", tmp);
                            serial.write(str.as_bytes()).unwrap();
                        }
                        'a' => {
                            let acc = mpu6050.read_acc().unwrap();
                            let str = format!(
                                "Planar acceleration: {:.2}, {:.2}, {:.2}\r\n",
                                acc.x, acc.y, acc.z
                            );
                            serial.write(str.as_bytes()).unwrap();
                        }
                        'g' => {
                            let gyro = mpu6050.read_gyro().unwrap();
                            let str = format!(
                                "Gyro acceleration: {:.2}, {:.2}, {:.2}\r\n",
                                gyro.x, gyro.y, gyro.z
                            );
                            serial.write(str.as_bytes()).unwrap();
                        }
                        'b' => {
                            serial.write("Initializing Adafruit 1893...\r\n".as_bytes()).unwrap();
                            match a1893.init(&mut delay) {
                                Ok(_) =>
                                    serial.write("Adafruit 1893 ok.\r\n".as_bytes()).unwrap(),
                                Err(Adafruit1893Error::I2c(e)) =>
                                    serial.write(format!("Adafruit 1893 I2C error: {:?}\r\n", e)
                                        .as_bytes()).unwrap(),
                                Err(Adafruit1893Error::InvalidChipId(x)) => {
                                    let str = format!("Adafruit 1893 whoami failed: {:#04x}\r\n", x);
                                    serial.write(str.as_bytes()).unwrap()
                                }
                                Err(Adafruit1893Error::NoResponse) =>
                                    serial.write("Adafruit 1893 no response\r\n".as_bytes()).unwrap()
                            };
                        },
                        'c' => {
                            motor_manager.turn_all_off();
                            serial.write("All motors off\r\n".as_bytes()).unwrap();
                        },
                        'm' => {
                            motor_manager.set_all_thrust_pct(0.05);
                            serial.write("All motors 5%\r\n".as_bytes()).unwrap();
                        },
                        '0' => {
                            motor_manager.m0().set_thrust_pct(0.05);
                            serial.write("M0 5%\r\n".as_bytes()).unwrap();
                        },
                        '1' => {
                            motor_manager.m1().set_thrust_pct(0.05);
                            serial.write("M1 5%\r\n".as_bytes()).unwrap();
                        },
                        '2' => {
                            motor_manager.m2().set_thrust_pct(0.05);
                            serial.write("M2 5%\r\n".as_bytes()).unwrap();
                        },
                        '3' => {
                            motor_manager.m3().set_thrust_pct(0.05);
                            serial.write("M3 5%\r\n".as_bytes()).unwrap();
                        },
                        _ => {}
                    });
                }
            }
        }
    }
}

fn setup_motors<LED: PinId>(
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

    let mut motor0 = Box::new(Motor::new_a(pwm0.channel_a, 20, p0));
    let mut motor1 = Box::new(Motor::new_b(pwm0.channel_b, 20, p1));
    let mut motor2 = Box::new(Motor::new_a(pwm1.channel_a, 20, p2));
    let mut motor3 = Box::new(Motor::new_b(pwm1.channel_b, 20, p3));
    let mut motor_manager = MotorManager::new([motor0, motor1, motor2, motor3]);
    motor_manager.setup(led, delay).unwrap();
    return motor_manager;
}

fn setup_mpu6050(
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

fn setup_adafruit1893(
    i2c0: I2C0,
    gpio8: Pin<Gpio8, PullDownDisabled>,
    gpio9: Pin<Gpio9, PullDownDisabled>,
    resets: &mut RESETS,
    system_clock: &SystemClock,
) -> Adafruit1893<I2C<I2C0, (Pin<Gpio8, FunctionI2C>, Pin<Gpio9, FunctionI2C>)>> {
    let mut a1893 = Adafruit1893::new(I2C::i2c0(
        i2c0,
        gpio8.into_mode(),
        gpio9.into_mode(),
        400.kHz(),
        resets,
        system_clock.freq().to_Hz().Hz(),
    ));
    a1893
}
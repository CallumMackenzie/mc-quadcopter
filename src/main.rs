#![no_std]
#![no_main]

mod drone;

extern crate alloc;

use alloc::format;

use cortex_m::delay::Delay;

use defmt_rtt as _;
use embedded_alloc::Heap;

use panic_halt as _;

use rp2040_hal::pwm::Slices;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
use rp_pico::{hal, Pins};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use adafruit1893_driver::Adafruit1893Error;

use crate::drone::{setup_adafruit1893, setup_motors, setup_mpu6050};

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// MUST BE CALLED BEFORE ALLOCATOR USED
fn init_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
}

#[rp2040_hal::entry]
fn main() -> ! {
    init_heap();

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
        .serial_number("Mackenzie-Drone")
        .device_class(USB_CLASS_CDC) // from: https://www.usb.org/defined-class-codes
        .build();

    // Init PWMs
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    let mut led = pins.led.into_push_pull_output();
    let mut motor_manager = setup_motors(
        &mut delay,
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
                            serial
                                .write("Initializing Adafruit 1893...\r\n".as_bytes())
                                .unwrap();
                            match a1893.init(&mut delay) {
                                Ok(_) => serial.write("Adafruit 1893 ok.\r\n".as_bytes()).unwrap(),
                                Err(Adafruit1893Error::I2c(e)) => serial
                                    .write(
                                        format!("Adafruit 1893 I2C error: {:?}\r\n", e).as_bytes(),
                                    )
                                    .unwrap(),
                                Err(Adafruit1893Error::InvalidChipId(x)) => {
                                    let str =
                                        format!("Adafruit 1893 whoami failed: {:#04x}\r\n", x);
                                    serial.write(str.as_bytes()).unwrap()
                                }
                                Err(Adafruit1893Error::NoResponse) => serial
                                    .write("Adafruit 1893 no response\r\n".as_bytes())
                                    .unwrap(),
                            };
                        }
                        'c' => {
                            motor_manager.turn_all_off();
                            serial.write("All motors off\r\n".as_bytes()).unwrap();
                        }
                        'm' => {
                            motor_manager.set_all_thrust_pct(0.05);
                            serial.write("All motors 5%\r\n".as_bytes()).unwrap();
                        }
                        '0' => {
                            motor_manager.m0().set_thrust_pct(0.05);
                            serial.write("M0 5%\r\n".as_bytes()).unwrap();
                        }
                        '1' => {
                            motor_manager.m1().set_thrust_pct(0.05);
                            serial.write("M1 5%\r\n".as_bytes()).unwrap();
                        }
                        '2' => {
                            motor_manager.m2().set_thrust_pct(0.05);
                            serial.write("M2 5%\r\n".as_bytes()).unwrap();
                        }
                        '3' => {
                            motor_manager.m3().set_thrust_pct(0.05);
                            serial.write("M3 5%\r\n".as_bytes()).unwrap();
                        }
                        _ => {}
                    });
                }
            }
        }
    }
}

#![no_std]
#![no_main]

use cortex_m::asm::delay;
use cortex_m::prelude::*;
use defmt::info;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use embedded_hal::PwmPin;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;
// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;
// GPIO traits
use rp_pico::hal::prelude::*;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM0
    let pwm = &mut pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.set_div_int(20u8); // 50 hz
    pwm.enable();

    // Output channel B on PWM0 to the GPIO1 pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio1);

    let ms_range = (0.5, 2.0);

    // 100% throttle
    channel.set_duty((channel.get_max_duty() as f32 / 20.0 * ms_range.1) as u16);
    for _ in 0..(5 * 5 * 2) {
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
    }

    // 0% throttle
    channel.set_duty((channel.get_max_duty() as f32 / 20.0 * ms_range.0) as u16);
    for _ in 0..(10 * 5) {
        led_pin.set_high().unwrap();
        delay.delay_ms(50);
        led_pin.set_low().unwrap();
        delay.delay_ms(50);
    }

    led_pin.set_low().unwrap();
    channel.set_duty((channel.get_max_duty() as f32 / 20.0 * 0.55) as u16);
    loop {}
}
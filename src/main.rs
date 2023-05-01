#![no_std]
#![no_main]

use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use rp2040_hal::gpio;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;

use cortex_m_rt::entry;
use embedded_time::rate::*;

#[entry]
fn main() -> ! {
    loop {}
}

#[allow(dead_code)]
fn radio_recieve_test() -> ! {
    use nrf24_rs::{
        config::{DataPipe, NrfConfig, PALevel},
        Nrf24l01,
        TransferError::{CommunicationError, MaximumRetries, Pin, Spi},
        SPI_MODE,
    };
    use rp_pico::hal::spi;

    // Retrieve resources
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
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
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let _spi_sclk = pins.gpio2.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio3.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<gpio::FunctionSpi>();
    let spi_cs = pins.gpio5.into_push_pull_output();

    let spi = spi::Spi::<_, _, 8>::new(pac.SPI0);
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let config = NrfConfig::default()
        .channel(8)
        .pa_level(PALevel::Min)
        .payload_size(4);

    let mut nrf_chip = Nrf24l01::new(spi, ce, ncs, &mut delay, config).unwrap();

    if !nrf_chip.is_connected().unwrap() {
        // Could not connect
    }
    nrf_chip.open_reading_pipe(DataPipe::DP0, b"drone").unwrap();
    nrf_chip.start_listening().unwrap();

    loop {
        while !nrf_chip.data_available().unwrap() {
            arduino_hal::delay_ms(50);
        }
        let mut buffer = [0; 4];
        nrf_chip.read(&mut buffer).unwrap();

        uprint!(&mut serial, "Got: {}", i32::from_be_bytes(buffer));
    }
}

#[allow(dead_code)]
fn radio_send_test() -> ! {
    use arduino_hal::spi;
    use nrf24_rs::{
        config::{NrfConfig, PALevel},
        Nrf24l01,
        TransferError::{CommunicationError, MaximumRetries, Pin, Spi},
        SPI_MODE,
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

    uprint!(&mut serial, "Pins retrieved");

    let settings = spi::Settings {
        data_order: spi::DataOrder::MostSignificantFirst,
        clock: spi::SerialClockRate::OscfOver4,
        mode: SPI_MODE,
    };
    uprint!(&mut serial, "Settins constructed");
    let (spi, ncs) = spi::Spi::new(dp.SPI, sck, mosi, miso, ss, settings);
    uprint!(&mut serial, "SPI & NCS constructed");

    let config = NrfConfig::default()
        .channel(8)
        .pa_level(PALevel::Min)
        .payload_size(4);
    uprint!(&mut serial, "Config created");

    let mut nrf_chip =
        Nrf24l01::new(spi, ce, ncs, &mut delay, config).unwrap_or_else(|e| match e {
            Spi(_) => upanic!(&mut serial, "SPI error"),
            Pin(_) => upanic!(&mut serial, "Serial error"),
            CommunicationError(v) => upanic!(&mut serial, "Communication error: {}", v),
            MaximumRetries => upanic!(&mut serial, "Max retries reached"),
        });

    if !nrf_chip.is_connected().unwrap() {
        upanic!(&mut serial, "Could not connect");
    }
    nrf_chip.open_writing_pipe(b"drone").unwrap();

    let mut ctr = 0i32;

    loop {
        while let Err(_) = nrf_chip.write(&mut delay, &ctr.to_be_bytes()) {
            arduino_hal::delay_ms(100);
            uprint!(&mut serial, "Could not transmit. Waiting...");
        }
        uprint!(&mut serial, "Sent!");
        ctr += 1;
        arduino_hal::delay_ms(100);
    }
}

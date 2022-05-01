#![no_std]

// use arduino_hal::spi::ChipSelectPin;
// use avr_hal_generic::port::PinOps;
// use core::convert::Infallible;
// use embedded_hal::digital::v2::OutputPin;

#[macro_export]
macro_rules! upanic {
	($($arg:tt)*) => {
		{
		loop {
			ufmt::uwriteln!($($arg)*).unwrap();
			arduino_hal::delay_ms(2000);
		}
		}
	};
}

#[macro_export]
macro_rules! uprint {
	($($arg:tt)*) => {
		ufmt::uwriteln!($($arg)*).unwrap();
	};
}
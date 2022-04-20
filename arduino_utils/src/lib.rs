#![no_std]

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

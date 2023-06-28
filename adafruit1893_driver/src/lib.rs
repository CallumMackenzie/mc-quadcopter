#![no_std]

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Write, WriteRead};

pub use consts::*;
use i2c_tools::{I2cDevice, I2cWrapperError};

mod consts;

pub struct Adafruit1893<T> {
    pub i2c: I2cDevice<T>,
}

impl<T, E> Adafruit1893<T>
where
    T: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: T) -> Self {
        Adafruit1893 {
            i2c: I2cDevice::new(i2c, ADAFRUIT1893_ADDR),
        }
    }

    pub fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Adafruit1893Error<E>> {
        self.i2c.whoami(WHOAMI::ADDR, WHOAMI::EXP_RESULT)?;
        self.reset_chip(delay)?;
        Ok(())
    }

    pub fn reset_chip<D: DelayMs<u8>>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Adafruit1893Error<E>> {
        self.i2c.write_byte(CTRL_REG1::ADDR, CTRL_REG1::RESET)?;
        let mut ctr = 0;
        while self.i2c.read_byte(CTRL_REG1::ADDR)? & CTRL_REG1::RESET != 0 && ctr < 10 {
            delay.delay_ms(10);
            ctr += 1;
        }
        if ctr >= 10 {
            Err(Adafruit1893Error::NoResponse)
        } else {
            Ok(())
        }
    }
}

#[derive(Debug)]
pub enum Adafruit1893Error<T> {
    I2c(T),
    InvalidChipId(u8),
    NoResponse,
}

impl<E> From<I2cWrapperError<E>> for Adafruit1893Error<E> {
    fn from(e: I2cWrapperError<E>) -> Self {
        match e {
            I2cWrapperError::I2c(x) => Adafruit1893Error::I2c(x),
            I2cWrapperError::InvalidChipId(x) => Adafruit1893Error::InvalidChipId(x),
        }
    }
}

#![no_std]

use embedded_hal::blocking::i2c::{Write, WriteRead};

pub use utils::*;

mod utils;

/// An i2c device with an address
pub struct I2cDevice<T> {
    i2c: T,
    slave_addr: u8,
}

impl<T, X> I2cDevice<T>
where
    T: Write<Error = X> + WriteRead<Error = X>,
{
    /// Creates a new I2C device wrapper
    pub fn new(i2c: T, slave_addr: u8) -> Self {
        I2cDevice { i2c, slave_addr }
    }

    /// Writes a single byte to the device at the provided register
    pub fn write_byte(&mut self, reg: u8, val: u8) -> Result<(), I2cWrapperError<X>> {
        self.i2c.write(self.slave_addr, &[reg, val])?;
        Ok(())
    }

    /// Reads a single byte from the device at the register provided
    pub fn read_byte(&mut self, reg: u8) -> Result<u8, I2cWrapperError<X>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c.write_read(self.slave_addr, &[reg], &mut byte)?;
        Ok(byte[0])
    }

    /// Reads bytes from the connected device into buff
    pub fn read_bytes(&mut self, reg: u8, buff: &mut [u8]) -> Result<(), I2cWrapperError<X>> {
        self.i2c.write_read(self.slave_addr, &[reg], buff)?;
        Ok(())
    }

    /// Writes a series of bits to the given register
    pub fn write_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
        data: u8,
    ) -> Result<(), I2cWrapperError<X>> {
        let mut byte = self.read_byte(reg)?;
        set_bits(&mut byte, start_bit, length, data);
        self.write_byte(reg, byte)?;
        Ok(())
    }

    /// Writes a single bit to a register
    pub fn write_bit(&mut self, reg: u8, bit: u8, value: bool) -> Result<(), I2cWrapperError<X>> {
        self.write_bits(reg, bit, 1, value as u8)
    }

    /// Verifies chip with provided addr
    pub fn whoami(&mut self, whoami_addr: u8, exp_value: u8) -> Result<(), I2cWrapperError<X>> {
        let addr = self.read_byte(whoami_addr)?;
        if addr != exp_value {
            Err(I2cWrapperError::InvalidChipId(addr))
        } else {
            Ok(())
        }
    }
}

pub enum I2cWrapperError<T> {
    I2c(T),
    InvalidChipId(u8),
}

impl<E> From<E> for I2cWrapperError<E> {
    fn from(e: E) -> Self {
        I2cWrapperError::I2c(e)
    }
}

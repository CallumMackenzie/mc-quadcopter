use embedded_hal::blocking::i2c::{Write, WriteRead};
use micromath::vector::F32x3;

const MPU_I2C_ADDR: u8 = 0x68;

#[derive(Debug)]
pub enum Mpu6050Error<T> {
    I2c(T),
    InvalidChipId(u8),
}

impl<E> From<E> for Mpu6050Error<E> {
    fn from(e: E) -> Self {
        Mpu6050Error::I2c(e)
    }
}

pub struct Mpu6050<'a, T> {
    i2c: &'a mut T,
    slave_addr: u8,
    acc_sensitivity: f32,
    gyro_sensitivity: f32,
}

impl<'a, T, E> Mpu6050<'a, T>
where
    T: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: &'a mut T) -> Self {
        Mpu6050 {
            i2c,
            slave_addr: MPU_I2C_ADDR,
            acc_sensitivity: AccelRange::G2.sensitivity(),
            gyro_sensitivity: GyroRange::D250.sensitivity(),
        }
    }

    pub fn wake(&mut self) -> Result<(), Mpu6050Error<E>> {
        self.write_byte(0x6b, 0x00)?;
        Ok(())
    }

    pub fn verify(&mut self) -> Result<(), Mpu6050Error<E>> {
        let addr = self.read_byte(0x75)?;
        if addr != MPU_I2C_ADDR {
            Err(Mpu6050Error::InvalidChipId(addr))
        } else {
            Ok(())
        }
    }

    pub fn init(&mut self) -> Result<(), Mpu6050Error<E>> {
        self.wake()?;
        self.verify()?;
        self.set_accel_range(AccelRange::G2)?;
        self.set_gyro_range(GyroRange::D250)?;
        Ok(())
    }

    pub fn get_temp(&mut self) -> Result<f32, Mpu6050Error<E>> {
        let mut buff: [u8; 2] = [0; 2];
        self.read_bytes(0x41, &mut buff)?;
        let raw_tmp = self.read_word_2c(&buff) as f32;
        Ok((raw_tmp / 340.0) + 36.53)
    }

    pub fn set_temp_enabled(&mut self, enabled: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(0x6b, 3, !enabled)?;
        Ok(())
    }

    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(0x1c, 4, 2, range as u8)?;
        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }

    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(0x1b, 4, 2, range as u8)?;
        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    fn read_rot(&mut self, reg: u8) -> Result<F32x3, Mpu6050Error<E>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf)?;

        Ok(F32x3::new(
            self.read_word_2c(&buf[0..2]) as f32,
            self.read_word_2c(&buf[2..4]) as f32,
            self.read_word_2c(&buf[4..6]) as f32,
        ))
    }

    fn write_byte(&mut self, reg: u8, val: u8) -> Result<(), Mpu6050Error<E>> {
        self.i2c.write(self.slave_addr, &[reg, val])?;
        Ok(())
    }

    fn read_byte(&mut self, reg: u8) -> Result<u8, Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c.write_read(self.slave_addr, &[reg], &mut byte)?;
        Ok(byte[0])
    }

    fn read_bytes(&mut self, reg: u8, buff: &mut [u8]) -> Result<(), Mpu6050Error<E>> {
        self.i2c.write_read(self.slave_addr, &[reg], buff)?;
        Ok(())
    }

    fn write_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
        data: u8,
    ) -> Result<(), Mpu6050Error<E>> {
        let mut byte = self.read_byte(reg)?;
        Self::set_bits(&mut byte, start_bit, length, data);
        self.write_byte(reg, byte)?;
        Ok(())
    }

    fn write_bit(&mut self, reg: u8, bit: u8, value: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(reg, bit, 1, value as u8)
    }

    fn set_bits(byte: &mut u8, bit_start: u8, length: u8, mut data: u8) {
        let mask_shift: u8 = if bit_start < length {
            0
        } else {
            bit_start - length + 1
        };
        let mask: u8 = ((1 << length) - 1) << mask_shift;
        data <<= mask_shift;
        data &= mask;
        *byte &= !(mask);
        *byte |= data;
    }

    fn read_word_2c(&self, byte: &[u8]) -> i32 {
        let high: i32 = byte[0] as i32;
        let low: i32 = byte[1] as i32;
        let mut word: i32 = (high << 8) + low;
        if word >= 0x8000 {
            word = -((65535 - word) + 1);
        }
        word
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum AccelRange {
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}

impl AccelRange {
    pub fn sensitivity(&self) -> f32 {
        match &self {
            AccelRange::G2 => 16384.0,
            AccelRange::G4 => 8192.0,
            AccelRange::G8 => 4096.0,
            AccelRange::G16 => 2048.0,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum GyroRange {
    D250 = 0,
    D500 = 1,
    D1000 = 2,
    D2000 = 3,
}

impl GyroRange {
    pub fn sensitivity(&self) -> f32 {
        match &self {
            GyroRange::D250 => 131.0,
            GyroRange::D500 => 65.5,
            GyroRange::D1000 => 32.8,
            GyroRange::D2000 => 16.4,
        }
    }
}

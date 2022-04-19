#![no_std]

pub mod consts;
mod utils;

pub use consts::*;
use core::f32::consts::PI;
use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Write, WriteRead},
};
use micromath::vector::{F32x2, F32x3};
use micromath::F32Ext;
use ufmt::derive::uDebug;
use utils::*;

pub struct Mpu6050<T, 'a> {
    i2c: T,                   // I2c bus driver
    slave_addr: u8,           // MPU slave addr
    acc_sensitivity: f32,     // Planar acceleration sensitivity
    gyro_sensitivity: f32,    // Gyroscope sensitivity
    pub gyro_err: F32x3,      // Gyroscopic acceleration error offset (degrees/sec)
    pub acc_angle_err: F32x2, // Planar acceleration angle error offset (g)
}

impl<T, E> Mpu6050<T>
where
    T: Write<Error = E> + WriteRead<Error = E>,
{
    // Creates a new mpu driver instance with the given i2c bus driver
    pub fn new(i2c: T) -> Self {
        Mpu6050 {
            i2c,
            slave_addr: MPU_ADDR,
            acc_sensitivity: AccelRange::G2.sensitivity(),
            gyro_sensitivity: GyroRange::D250.sensitivity(),
            gyro_err: f32x3_empty(),
            acc_angle_err: f32x2_empty(),
        }
    }

    // Wakes the mpu
    pub fn wake<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>> {
        self.write_byte(PWR_MGMT_1::ADDR, 0x01)?;
        delay.delay_ms(100);
        Ok(())
    }

    // Resets the mpu
    pub fn reset_device<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::RESET_BIT, true)?;
        delay.delay_ms(100);
        Ok(())
    }

    // Verifies the integrity of the mpu
    pub fn verify(&mut self) -> Result<(), Mpu6050Error<E>> {
        let addr = self.read_byte(WHO_AM_I::ADDR)?;
        if addr != MPU_ADDR {
            Err(Mpu6050Error::InvalidChipId(addr))
        } else {
            Ok(())
        }
    }

    // Initializes the mpu
    pub fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Mpu6050Error<E>> {
        self.reset_device(delay)?;
        self.wake(delay)?;
        self.verify()?;
        self.set_accel_range(AccelRange::G2)?;
        self.set_gyro_range(GyroRange::D250)?;
        Ok(())
    }

    // Reads temperature
    pub fn read_temp(&mut self) -> Result<f32, Mpu6050Error<E>> {
        const nbytes: u8 = TEMP_OUT::BYTES.len;
        let mut buff: [u8; nbytes] = [0; nbytes];
        self.read_bytes(TEMP_OUT::ADDR, &mut buff)?;
        let raw_tmp = self.read_word_2c(&buff) as f32;
        Ok((raw_tmp / TEMP_SENSITIVITY) + TEMP_OFFSET)
    }

    // Enables or disables temperature measurement
    pub fn set_temp_enabled(&mut self, enabled: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS_BIT, !enabled)?;
        Ok(())
    }

    // Sets the acceleration measurement range
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Mpu6050Error<E>> {
        const bits: BitBlock = ACCEL_CONFIG::ACCEL_FS_SEL_BITS;
        self.write_bits(ACCEL_CONFIG::ADDR, bits.start, bits.len, range as u8)?;
        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }

    // Sets the gyro measurement range
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Mpu6050Error<E>> {
        const bits: BitBlock = GYRO_CONFIG::GYRO_FS_SEL_BITS;
        self.write_bits(GYRO_CONFIG::ADDR, bits.start, bits.len, 2, range as u8)?;
        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    // Calculates all error offset values. Device should be placed flat and not moving.
    pub fn calculate_all_imu_error(&mut self, iters: i32) -> Result<(), Mpu6050Error<E>> {
        self.calculate_imu_acc_angle_error(iters)?;
        self.calculate_imu_gyro_error(iters)?;
        Ok(())
    }

    // Calculates gyroscope error offset values based on current readings.
    // Device should be placed flat and not be moving.
    pub fn calculate_imu_gyro_error(&mut self, iters: i32) -> Result<(), Mpu6050Error<E>> {
        let mut gyro = f32x3_empty();
        self.gyro_err = f32x3_empty();
        for _ in 0..iters {
            self.read_gyro_raw_to_ref(&mut gyro)?;
            self.gyro_err.x += gyro.x;
            self.gyro_err.y += gyro.y;
            self.gyro_err.z += gyro.z;
        }
        self.gyro_err.x /= iters as f32;
        self.gyro_err.y /= iters as f32;
        self.gyro_err.z /= iters as f32;
        Ok(())
    }

    // Calculates acceleration angle error offset values based on current readings.
    // Device should be placed flat and not be moving.
    pub fn calculate_imu_acc_angle_error(&mut self, iters: i32) -> Result<(), Mpu6050Error<E>> {
        let mut acc = f32x3_empty();
        let mut tmp = f32x2_empty();
        self.acc_angle_err = f32x2_empty();
        for _ in 0..iters {
            self.read_acc_to_ref(&mut acc)?;
            Self::calc_acc_angle_raw(&acc, &mut tmp);
            self.acc_angle_err.x += tmp.x;
            self.acc_angle_err.y += tmp.y;
        }
        self.acc_angle_err.x /= iters as f32;
        self.acc_angle_err.y /= iters as f32;
        Ok(())
    }

    // Reads acceleration angle (roll & pitch)
    pub fn read_acc_angle_to_ref(&mut self, dst: &mut F32x2) -> Result<(), Mpu6050Error<E>> {
        let acc = self.read_acc()?;
        Self::calc_acc_angle_to_ref(&acc, &self.acc_angle_err, dst);
        Ok(())
    }

    // Reads acceleration angle (roll & pitch)
    pub fn read_acc_angle(&mut self) -> Result<F32x2, Mpu6050Error<E>> {
        let mut ret = f32x2_empty();
        self.read_acc_angle_to_ref(&mut ret)?;
        Ok(ret)
    }

    // Reads gyroscopic acceleration (deg/s), accounting for calibrated error
    pub fn read_gyro(&mut self) -> Result<F32x3, Mpu6050Error<E>> {
        let mut ret = f32x3_empty();
        self.read_gyro_to_ref(&mut ret)?;
        Ok(ret)
    }

    // Reads gyroscopic acceleration (deg/s), accounting for calibrated error
    pub fn read_gyro_to_ref(&mut self, dst: &mut F32x3) -> Result<(), Mpu6050Error<E>> {
        self.read_gyro_raw_to_ref(dst)?;
        dst.x -= self.gyro_err.x;
        dst.y -= self.gyro_err.y;
        dst.z -= self.gyro_err.z;
        Ok(())
    }

    // Reads gyroscopic acceleration (deg/s)
    pub fn read_gyro_raw(&mut self) -> Result<F32x3, Mpu6050Error<E>> {
        let mut ret = f32x3_empty();
        self.read_gyro_raw_to_ref(&mut ret)?;
        Ok(ret)
    }

    // Reads gyroscopic acceleration (deg/s)
    pub fn read_gyro_raw_to_ref(&mut self, dst: &mut F32x3) -> Result<(), Mpu6050Error<E>> {
        self.read_f32x3(GYRO_OUT::ADDR, dst)?;
        dst.x /= self.gyro_sensitivity;
        dst.y /= self.gyro_sensitivity;
        dst.z /= self.gyro_sensitivity;
        Ok(())
    }

    // Reads planar acceleration (Gs)
    pub fn read_acc(&mut self) -> Result<F32x3, Mpu6050Error<E>> {
        let mut ret = f32x3_empty();
        self.read_acc_to_ref(&mut ret)?;
        Ok(ret)
    }

    // Reads planar acceleration (Gs)
    pub fn read_acc_to_ref(&mut self, dst: &mut F32x3) -> Result<(), Mpu6050Error<E>> {
        self.read_f32x3(ACCEL_OUT::ADDR, dst)?;
        dst.x /= self.acc_sensitivity;
        dst.y /= self.acc_sensitivity;
        dst.z /= self.acc_sensitivity;
        Ok(())
    }

    // Calculates roll & pitch from acceleration data
    fn calc_acc_angle_raw(acc: &F32x3, dst: &mut F32x2) {
        dst.x = (acc.y / (acc.x * acc.x + acc.z * acc.z).sqrt()).atan() * 180.0 / PI;
        dst.y = (-acc.x / (acc.y * acc.y + acc.z * acc.z).sqrt()).atan() * 180.0 / PI;
    }

    // Calculates roll & pitch from acceleration data
    pub fn calc_acc_angle_to_ref(acc: &F32x3, err: &F32x2, dst: &mut F32x2) {
        Self::calc_acc_angle_raw(acc, dst);
        dst.x += err.x;
        dst.y += err.y;
    }

    // Calculates roll & pitch from acceleration data
    pub fn calc_acc_angle(acc: &F32x3, err: &F32x2) -> F32x2 {
        let mut ret = f32x2_empty();
        Self::calc_acc_angle_to_ref(acc, err, &mut ret);
        ret
    }

    fn read_f32x3(&mut self, reg: u8, dst: &mut F32x3) -> Result<(), Mpu6050Error<E>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf)?;
        dst.x = self.read_word_2c(&buf[0..2]) as f32;
        dst.y = self.read_word_2c(&buf[2..4]) as f32;
        dst.z = self.read_word_2c(&buf[4..6]) as f32;
        Ok(())
    }

    // Writes a single byte to mpu
    pub fn write_byte(&mut self, reg: u8, val: u8) -> Result<(), Mpu6050Error<E>> {
        self.i2c.write(self.slave_addr, &[reg, val])?;
        Ok(())
    }

    // Reads a single byte from mpu
    pub fn read_byte(&mut self, reg: u8) -> Result<u8, Mpu6050Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c.write_read(self.slave_addr, &[reg], &mut byte)?;
        Ok(byte[0])
    }

    // Reades bytes from mpu
    pub fn read_bytes(&mut self, reg: u8, buff: &mut [u8]) -> Result<(), Mpu6050Error<E>> {
        self.i2c.write_read(self.slave_addr, &[reg], buff)?;
        Ok(())
    }

    // Writes bits to mpu
    pub fn write_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
        data: u8,
    ) -> Result<(), Mpu6050Error<E>> {
        let mut byte = self.read_byte(reg)?;
        set_bits(&mut byte, start_bit, length, data);
        self.write_byte(reg, byte)?;
        Ok(())
    }

    // Writes a single bit to mpu
    pub fn write_bit(&mut self, reg: u8, bit: u8, value: bool) -> Result<(), Mpu6050Error<E>> {
        self.write_bits(reg, bit, 1, value as u8)
    }
}

#[derive(Debug, uDebug)]
pub enum Mpu6050Error<T> {
    I2c(T),
    InvalidChipId(u8),
}

impl<E> From<E> for Mpu6050Error<E> {
    fn from(e: E) -> Self {
        Mpu6050Error::I2c(e)
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone, uDebug)]
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

#[derive(Debug, Eq, PartialEq, Copy, Clone, uDebug)]
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

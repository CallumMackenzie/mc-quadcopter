use core::f32::consts::PI;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use micromath::vector::{F32x2, F32x3};
use micromath::F32Ext;
use ufmt::derive::uDebug;

const MPU_I2C_ADDR: u8 = 0x68;

fn f32x2_empty() -> F32x2 {
    F32x2 { x: 0.0, y: 0.0 }
}
fn f32x3_empty() -> F32x3 {
    F32x3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
}

#[derive(Debug, uDebug)]
pub enum Mpu6050Error<T> {
    I2c(T),
    InvalidChipId(u8),
    CouldNotUnhang,
}

impl<E> From<E> for Mpu6050Error<E> {
    fn from(e: E) -> Self {
        Mpu6050Error::I2c(e)
    }
}

pub struct Mpu6050<'a, T> {
    i2c: &'a mut T,        // I2c bus driver
    slave_addr: u8,        // MPU slave addr
    acc_sensitivity: f32,  // Planar acceleration sensitivity
    gyro_sensitivity: f32, // Gyroscope sensitivity
    pub gyro_err: F32x3,   // Gyroscopic acceleration error offset (degrees/sec)
    pub acc_err: F32x2,    // Planar acceleration angle error offset (g)
}

impl<'a, T: 'a> Mpu6050<'a, T> {
    fn calc_acc_angle_raw(acc: &F32x3, dst: &mut F32x2) {
        dst.x = (acc.y / (acc.x * acc.x + acc.z * acc.z).sqrt()).atan() * 180.0 / PI;
        dst.y = (-acc.x / (acc.y * acc.y + acc.z * acc.z).sqrt()).atan() * 180.0 / PI;
    }
    fn calc_acc_angle(acc: &F32x3, err: &F32x2, dst: &mut F32x2) {
        Self::calc_acc_angle_raw(acc, dst);
        dst.x += err.x;
        dst.y += err.y;
    }
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
            gyro_err: f32x3_empty(),
            acc_err: f32x2_empty(),
        }
    }

    pub fn wake(&mut self) -> Result<(), Mpu6050Error<E>> {
        self.write_byte(0x6b, 0x01)?;
        arduino_hal::delay_ms(100);
        Ok(())
    }

    pub fn reset_device(&mut self) -> Result<(), Mpu6050Error<E>> {
        self.write_bit(0x6b, 7, true)?;
        arduino_hal::delay_ms(100);
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
        self.reset_device()?;
        self.wake()?;
        self.verify()?;
        self.set_accel_range(AccelRange::G2)?;
        self.set_gyro_range(GyroRange::D250)?;
        Ok(())
    }

    pub fn read_temp(&mut self) -> Result<f32, Mpu6050Error<E>> {
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

    // pub fn read_all_positional(&mut self) -> Result<(), Mpu6050Error<E>> {
    //     // Retrieve elapsed time
    //     let now = (self.millis)();
    //     let elapsed_time = (now - self.last_gyro_measurement) as f32 / 1000.0;
    //     self.last_gyro_measurement = now;
    //     // Read acceleration
    //     let mut tmp = f32x3_empty();
    //     self.read_acc_to_ref(&mut tmp)?;
    //     self.acc = tmp;
    //     // Calculate acceleration angle
    //     let mut tmp = f32x2_empty();
    //     Self::calc_acc_angle(&self.acc, &self.acc_err, &mut tmp);
    //     self.acc_angle = tmp;

    //     // Read gyro
    //     let mut tmp = f32x3_empty();
    //     self.read_gyro_raw_to_ref(&mut tmp)?;
    //     self.gyro = tmp;
    //     self.gyro.x += self.gyro_err.x;
    //     self.gyro.y += self.gyro_err.y;
    //     self.gyro.z += self.gyro_err.z;
    //     // Calculate gyro angles
    //     self.gyro_angle.x += self.gyro.x * elapsed_time;
    //     self.gyro_angle.y += self.gyro.x * elapsed_time;
    //     // Calculate roll, pitch, and yaw
    //     self.rotation.x =
    //         self.rotation_filter * self.gyro.x + (1.0 - self.rotation_filter) * self.acc_angle.x; // Roll
    //     self.rotation.y =
    //         self.rotation_filter * self.gyro.y + (1.0 - self.rotation_filter) * self.acc_angle.y; // Pitch
    //     self.rotation.z += self.gyro.z * elapsed_time; // Yaw

    //     Ok(())
    // }

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
        self.acc_err = f32x2_empty();
        for _ in 0..iters {
            self.read_acc_to_ref(&mut acc)?;
            Self::calc_acc_angle_raw(&acc, &mut tmp);
            self.acc_err.x += tmp.x;
            self.acc_err.y += tmp.y;
        }
        self.acc_err.x /= iters as f32;
        self.acc_err.y /= iters as f32;
        Ok(())
    }
    
    // Reads acceleration angle (roll & pitch)
    pub fn read_acc_angle_to_ref(&mut self, dst: &mut F32x2) -> Result<(), Mpu6050Error<E>> {
        let acc = self.read_acc()?;
        Self::calc_acc_angle(&acc, &self.acc_err, dst);
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
        self.read_f32x3(0x43, dst)?;
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
        self.read_f32x3(0x3b, dst)?;
        dst.x /= self.acc_sensitivity;
        dst.y /= self.acc_sensitivity;
        dst.z /= self.acc_sensitivity;
        Ok(())
    }

    fn read_f32x3(&mut self, reg: u8, dst: &mut F32x3) -> Result<(), Mpu6050Error<E>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf)?;
        dst.x = self.read_word_2c(&buf[0..2]) as f32;
        dst.y = self.read_word_2c(&buf[2..4]) as f32;
        dst.z = self.read_word_2c(&buf[4..6]) as f32;
        Ok(())
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

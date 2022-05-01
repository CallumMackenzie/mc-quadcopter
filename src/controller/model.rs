use millis_driver::millis;
use elinalgebra::{F32x2, F32x3};
use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Write, WriteRead},
};
use mpu6050_driver::{Mpu6050, Mpu6050Error};
use ufmt::derive::uDebug;

pub trait PositionInput<E> {
    type Error;

    fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Self::Error>;
    fn calibrate(&mut self) -> Result<(), Self::Error>;
    fn read(&mut self, pos_data: &mut PositionData) -> Result<(), Self::Error>;
}

pub struct PositionData {
    /// Last measurement time (in ms since start of program)
    pub last_gyro_measurement: u32,
    /// Planar acceleration in Gs (9.81 m\s)
    pub acc: F32x3,
    /// Gyroscopic acceleration in degrees per second (roll, pitch, yaw)
    pub gyro_acc: F32x3,
    /// Raw gyroscopic acceleration angle (roll, pitch, yaw)
    pub gyro_angle: F32x3,
    /// Raw acceleration-derived gyroscopic angle (roll, pitch)
    pub acc_angle: F32x2,
}

impl PositionData {
    pub fn new() -> Self {
        PositionData {
            last_gyro_measurement: 0,
            acc: F32x3::filled(0.0),
            gyro_acc: F32x3::filled(0.0),
            gyro_angle: F32x3::filled(0.0),
            acc_angle: F32x2::filled(0.0),
        }
    }
}

#[derive(Debug, uDebug)]
pub enum PosInputErr<E> {
    DriverError(E),
    UnknownDriverError,
}

pub struct MpuPositionInput<T> {
    /// The mpu6050 inertial measurement driver
    pub mpu: Mpu6050<T>,
}

impl<T, E> MpuPositionInput<T>
where
    T: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: T) -> Self {
        Self {
            mpu: Mpu6050::new(i2c),
        }
    }
}

impl<T, E> PositionInput<E> for MpuPositionInput<T>
where
    T: Write<Error = E> + WriteRead<Error = E>,
{
    type Error = PosInputErr<E>;

    fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Self::Error> {
        self.mpu.init(delay)?;
        Ok(())
    }

    fn calibrate(&mut self) -> Result<(), Self::Error> {
        self.mpu.calculate_all_imu_error(150)?;
        Ok(())
    }

    fn read(&mut self, p: &mut PositionData) -> Result<(), Self::Error> {
        // Retrieve elapsed time
        let now = millis();
        let elapsed_time = (now - p.last_gyro_measurement) as f32 / 1000.0;
        p.last_gyro_measurement = now;
        // Read acceleration
        p.acc = self.mpu.read_acc()?;
        // Calculate acceleration angle
        p.acc_angle = Mpu6050::<T>::calc_acc_angle(&p.acc, &self.mpu.acc_angle_err);
        // Read gyro
        p.gyro_acc = self.mpu.read_gyro()?;
        // Calculate gyro angles
        p.gyro_angle += p.gyro_acc * elapsed_time;
        Ok(())
    }
}

impl<E> From<Mpu6050Error<E>> for PosInputErr<E> {
    fn from(e: Mpu6050Error<E>) -> Self {
        use Mpu6050Error::*;
        match e {
            I2c(err) => PosInputErr::DriverError(err),
            InvalidChipId(_) => PosInputErr::UnknownDriverError,
        }
    }
}

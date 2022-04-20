use arduino_millis_driver::millis;
use elinalgebra::{F32x2, F32x3};
use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Write, WriteRead},
};
use mpu6050_driver::{Mpu6050, Mpu6050Error};
use ufmt::derive::uDebug;

pub struct PositionInput<T> {
    /// The mpu6050 inertial measurement driver
    pub mpu: Mpu6050<T>,
    /// Last measurement time (in ms since start of program)
    pub last_gyro_measurement: u32,
    /// Planar acceleration in Gs (9.81 m\s)
    pub acc: F32x3,
    /// Gyroscopic acceleration in degrees per second (roll, pitch, yaw)
    pub gyro_acc: F32x3,
    /// Filter for combining rotation from gyroscope and rotation derived from acceleration.
    /// Must be between 0 and 1 (inclusive).
    pub rotation_filter: f32,
    /// Current rotation in degrees (roll, pitch, yaw)
    pub rotation: F32x3,
    /// Raw gyroscopic acceleration angle (roll, pitch, yaw)
    pub gyro_angle: F32x3,
    /// Raw acceleration-derived gyroscopic angle (roll, pitch)
    pub acc_angle: F32x2,
}

impl<T, E> PositionInput<T>
where
    T: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: T) -> Self {
        Self::from_mpu(Mpu6050::new(i2c))
    }

    pub fn from_mpu(mpu: Mpu6050<T>) -> Self {
        PositionInput {
            mpu,
            last_gyro_measurement: 0,
            acc: F32x3::filled(0.0),
            gyro_acc: F32x3::filled(0.0),
            rotation_filter: 0.96,
            rotation: F32x3::filled(1.0),
            gyro_angle: F32x3::filled(0.0),
            acc_angle: F32x2::filled(0.0),
        }
    }

    pub fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), PosInputErr<E>> {
        self.mpu.init(delay)?;
        self.last_gyro_measurement = millis();
        Ok(())
    }

    pub fn calibrate(&mut self) -> Result<(), PosInputErr<E>> {
        self.gyro_angle = F32x3::filled(0.0);
        self.acc_angle = F32x2::filled(0.0);
        self.mpu.calculate_all_imu_error(150)?;
        self.last_gyro_measurement = millis();
        Ok(())
    }

    pub fn read(&mut self) -> Result<(), Mpu6050Error<E>> {
        // Retrieve elapsed time
        let now = millis();
        let elapsed_time = (now - self.last_gyro_measurement) as f32 / 1000.0;
        self.last_gyro_measurement = now;
        // Read acceleration
        self.acc = self.mpu.read_acc()?;
        // Calculate acceleration angle
        self.acc_angle = Mpu6050::<T>::calc_acc_angle(&self.acc, &self.mpu.acc_angle_err);

        // Read gyro
        self.gyro_acc = self.mpu.read_gyro()?;
        // Calculate gyro angles
        self.gyro_angle += self.gyro_acc * elapsed_time;
        // Calculate roll, pitch, and yaw
        let (gyro_filter, acc_filter): (f32, f32) = (self.rotation_filter, 1f32 - self.rotation_filter);
        self.rotation.x = self.gyro_angle.x * gyro_filter + self.acc_angle.x * acc_filter; // Roll
        self.rotation.y = self.gyro_angle.y * gyro_filter + self.acc_angle.y * acc_filter; // Pitch
        self.rotation.z = self.gyro_angle.z; // Yaw
        Ok(())
    }
}

#[derive(Debug, uDebug)]
pub enum PosInputErr<E> {
    MpuError(Mpu6050Error<E>),
}

impl<E> From<Mpu6050Error<E>> for PosInputErr<E> {
    fn from(e: Mpu6050Error<E>) -> Self {
        PosInputErr::MpuError(e)
    }
}

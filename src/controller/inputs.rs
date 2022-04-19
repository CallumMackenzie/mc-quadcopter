pub mod millis;
pub mod mpu6050;

use embedded_hal::blocking::i2c::{Write, WriteRead};
use micromath::vector::{F32x2, F32x3};
use millis::millis;
use mpu6050::*;
use ufmt::derive::uDebug;

pub struct PositionInput<T> {
    pub mpu: Mpu6050<T>,            // The mpu6050 inertial measurement driver
    pub last_gyro_measurement: u32, // Last measurement time (in ms since start of program)
    pub acc: F32x3,                 // Planar acceleration in Gs (9.81 m\s)
    pub gyro: F32x3, // Gyroscopic acceleration in degrees per second (roll, pitch, yaw)
    pub rotation_filter: f32, // Filter for combining rotation from gyroscope and rotation derived from acceleration
    pub rotation: F32x3,      // Current rotation in degrees (roll, pitch, yaw)
    gyro_angle: F32x3,        // Raw gyroscopic angle (roll, pitch, yaw)
    acc_angle: F32x2,         // Raw acceleration angle (roll, pitch)
}

impl<T, E> PositionInput<T>
where
    T: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: T) -> Self {
        Self::new(Mpu6050::new(i2c))
    }

    pub fn from_mpu(mpu: Mpu6050<T>) -> Self {
        PositionInput {
            mpu,
            last_gyro_measurement: 0,
            acc: f32x3_empty(),
            gyro: f32x3_empty(),
            rotation_filter: 0.96,
            rotation: f32x3_empty(),
            gyro_angle: f32x3_empty(),
            acc_angle: f32x2_empty(),
        }
    }

    pub fn init(&mut self) -> Result<(), PosInputErr<E>> {
        self.mpu.init()?;
        self.last_gyro_measurement = millis();
        Ok(())
    }

    pub fn calibrate(&mut self) -> Result<(), PosInputErr<E>> {
        self.mpu.calculate_all_imu_error(150)?;
        Ok(())
    }

    pub fn update(&mut self) -> Result<(), Mpu6050Error<E>> {
        // Retrieve elapsed time
        let now = millis();
        let elapsed_time = (now - self.last_gyro_measurement) as f32 / 1000.0;
        self.last_gyro_measurement = now;
        // Read acceleration
        self.acc = self.mpu.read_acc()?;
        // Calculate acceleration angle
        self.acc_angle = Mpu6050::<T>::calc_acc_angle(&self.acc, &self.mpu.acc_angle_err);

        // Read gyro
        self.gyro = self.mpu.read_gyro()?;
        // Calculate gyro angles
        self.gyro_angle.x += self.gyro.x * elapsed_time;
        self.gyro_angle.y += self.gyro.x * elapsed_time;
        self.gyro_angle.z += self.gyro.z * elapsed_time;
        // Calculate roll, pitch, and yaw
        self.rotation.x =
            self.rotation_filter * self.gyro.x + (1.0 - self.rotation_filter) * self.acc_angle.x; // Roll
        self.rotation.y =
            self.rotation_filter * self.gyro.y + (1.0 - self.rotation_filter) * self.acc_angle.y; // Pitch
        self.rotation.z += self.gyro.z * elapsed_time; // Yaw

        // We're done
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

#![no_std]

use cortex_m::delay::Delay;
use cortex_m::prelude::*;
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use rp2040_hal::gpio::{Pin, PinId, PinMode, PushPullOutput, ValidPinMode};
use rp2040_hal::gpio::bank0::BankPinId;
use rp2040_hal::pwm::{A, B, Channel, ChannelId, SliceId, SliceMode, ValidPwmOutputPin, ValidSliceMode};

/// A motor with a given PWM channel
pub struct Motor<'a, S, M, C>
    where S: SliceId,
          M: SliceMode + ValidSliceMode<S>,
          C: ChannelId
{
    channel: &'a mut Channel<S, M, C>,
    initialized: bool,
    duty_range: (u16, u16),
}


/// A trait representing a motor which has been setup with its channel
/// and pin.
pub trait SetupMotor {
    /// REQUIRES: pct in [0,1]
    /// EFFECTS: Sets motor thrust percentage
    fn set_thrust_pct(&mut self, pct: f32);
    /// EFFECTS: Returns whether this motor has been initialized
    fn is_initialized(&self) -> bool;
    /// EFFECTS: Marks this motor as initialized
    fn mark_initialized(&mut self);
}

impl<'a, S, M> Motor<'a, S, M, A>
    where S: SliceId,
          M: SliceMode + ValidSliceMode<S>
{
    /// REQUIRES: Output is a valid pin for the given channel
    /// EFFECTS: Sets the channel output and returns a motor struct wrapper
    pub fn new_a<P, PM>(channel: &'a mut Channel<S, M, A>,
                        div_int: u8,
                        output: Pin<P, PM>)
                        -> Motor<'a, S, M, A>
        where
            P: PinId + BankPinId + ValidPwmOutputPin<S, A>,
            PM: PinMode + ValidPinMode<P>,
    {
        channel.output_to(output);
        let duty_per_ms = channel.get_max_duty() as f32 / div_int as f32;
        Motor {
            channel,
            initialized: false,
            duty_range: ((duty_per_ms * 0.5) as u16, (duty_per_ms * 2.0) as u16),
        }
    }
}

impl<'a, S, M> Motor<'a, S, M, B>
    where S: SliceId,
          M: SliceMode + ValidSliceMode<S>
{
    /// REQUIRES: Output is a valid pin for the given channel
    /// EFFECTS: Sets the channel output and returns a motor struct wrapper
    pub fn new_b<P, PM>(channel: &'a mut Channel<S, M, B>,
                        div_int: u8,
                        output: Pin<P, PM>)
                        -> Motor<'a, S, M, B>
        where
            P: PinId + BankPinId + ValidPwmOutputPin<S, B>,
            PM: PinMode + ValidPinMode<P>,
    {
        channel.output_to(output);
        let duty_per_ms = channel.get_max_duty() as f32 / div_int as f32;
        Motor {
            channel,
            initialized: false,
            duty_range: ((duty_per_ms * 0.5) as u16, (duty_per_ms * 2.0) as u16),
        }
    }
}

impl<'a, S, M> SetupMotor for Motor<'a, S, M, A>
    where S: SliceId,
          M: SliceMode + ValidSliceMode<S>,
{
    fn set_thrust_pct(&mut self, pct: f32) {
        let duty = ((self.duty_range.1 - self.duty_range.0) as f32 * pct)
            as u16 + self.duty_range.0;
        self.channel.set_duty(duty);
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }

    fn mark_initialized(&mut self) {
        self.initialized = true;
    }
}

impl<'a, S, M> SetupMotor for Motor<'a, S, M, B>
    where S: SliceId,
          M: SliceMode + ValidSliceMode<S>,
{
    fn set_thrust_pct(&mut self, pct: f32) {
        let duty = ((self.duty_range.1 - self.duty_range.0) as f32 * pct)
            as u16 + self.duty_range.0;
        self.channel.set_duty(duty);
    }

    fn is_initialized(&self) -> bool {
        self.initialized
    }

    fn mark_initialized(&mut self) {
        self.initialized = true;
    }
}

pub struct MotorManager<'a> {
    pub motors: &'a mut [&'a mut dyn SetupMotor],
}

impl<'a> MotorManager<'a> {
    pub fn setup<I: PinId>(&mut self,
                           indicator_led: &mut Pin<I, PushPullOutput>,
                           delay: &mut Delay) -> Result<(), MotorError> {
        for motor in self.motors.iter_mut() {
            if motor.is_initialized() {
                return Err(MotorError::AlreadyInitialized);
            }
            motor.set_thrust_pct(1.0);
        }
        // Delay 1500 ms
        for _ in 0..15 {
            indicator_led.toggle().unwrap();
            delay.delay_ms(100);
        }
        for motor in self.motors.iter_mut() {
            motor.set_thrust_pct(0.0);
        }
        // Delay 2000 ms
        for _ in 0..40 {
            indicator_led.toggle().unwrap();
            delay.delay_ms(50);
        }
        for motor in self.motors.iter_mut() {
            motor.mark_initialized();
        }
        indicator_led.set_high().unwrap();
        Ok(())
    }

    pub fn set_all_thrust_pct(&mut self, pct: f32) {
        for motor in self.motors.iter_mut() {
            motor.set_thrust_pct(pct);
        }
    }

    pub fn get_motor_count(&self) -> usize {
        self.motors.len()
    }
}

#[derive(Debug, Copy, Clone)]
pub enum MotorError {
    AlreadyInitialized
}

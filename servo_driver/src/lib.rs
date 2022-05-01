#![no_std]

//! Servo driver interfaces for the arduino uno

use arduino_hal::port::{mode::Output, Pin};
use atmega_hal::{
    pac::TC2,
    port::{PB3, PD3},
};
use avr_hal_generic::port::PinOps;
use core::cell::RefCell;

/// A generic limited servo driver
pub trait ServoDriver {
    /// Set servo angle in degrees: (-45 to 45)
    fn set_angle(&mut self, angle: f32);
}

pub trait ServoDriverDelegate {
    /// Set raw duty
    fn set_duty(&mut self, duty: u8);
}

const TRAXXAS_DUTY_RANGE: [f32; 2] = [15f32, 32f32];

#[allow(dead_code)]
pub struct Traxxas2080Servo<PIN: PinOps, TIMER> {
    pin: Pin<Output, PIN>,
    tc: *mut TIMER,
}

impl Traxxas2080Servo<PD3, TC2> {
    pub fn pd3(tc2: &RefCell<TC2>, pin: Pin<Output, PD3>) -> Self {
        enable_pwm_3_11(tc2.as_ptr());
        Self {
            pin: pin,
            tc: tc2.as_ptr(),
        }
    }
}

impl Traxxas2080Servo<PB3, TC2> {
    pub fn pb3(tc2: &RefCell<TC2>, pin: Pin<Output, PB3>) -> Self {
        enable_pwm_3_11(tc2.as_ptr());
        Self {
            pin: pin,
            tc: tc2.as_ptr(),
        }
    }
}

impl<PIN: PinOps, TIMER> ServoDriver for Traxxas2080Servo<PIN, TIMER>
where
    Traxxas2080Servo<PIN, TIMER>: ServoDriverDelegate,
{
    fn set_angle(&mut self, mut angle: f32) {
        if angle > 45.0 {
            angle = 45.0;
        } else if angle < -45.0 {
            angle = -45.0;
        }
        let duty = ((angle + 45.0) / 90.0) * (TRAXXAS_DUTY_RANGE[1] - TRAXXAS_DUTY_RANGE[0])
            + TRAXXAS_DUTY_RANGE[0];
        self.set_duty(duty as u8);
    }
}

impl ServoDriverDelegate for Traxxas2080Servo<PD3, TC2> {
    fn set_duty(&mut self, duty: u8) {
        unsafe {
            (*(self.tc)).ocr2b.write(|w| w.bits(duty));
        }
    }
}

impl ServoDriverDelegate for Traxxas2080Servo<PB3, TC2> {
    fn set_duty(&mut self, duty: u8) {
        unsafe {
            (*(self.tc)).ocr2a.write(|w| w.bits(duty));
        }
    }
}

fn enable_pwm_3_11(tc2: *mut TC2) {
    unsafe {
        (*tc2).tccr2a.write(|w| {
            w.wgm2()
                .pwm_fast()
                .com2a()
                .match_clear()
                .com2b()
                .match_clear()
        });
        (*tc2).tccr2b.write(|w| w.cs2().prescale_1024());
    }
}

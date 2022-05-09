#![no_std]

//! Servo driver interfaces for the arduino uno
//! See [this site](https://www.arduino.cc/en/pmwiki.php?n=Tutorial/SecretsOfArduinoPWM) for info on
//! implementation.

use arduino_hal::port::{mode::Output, Pin};
use atmega_hal::{
    pac::{TC0, TC1, TC2},
    port::{PB1, PB2, PB3, PD3, PD5, PD6},
};
use avr_hal_generic::port::PinOps;
use core::cell::RefCell;
use millis_driver::millis_initialized;

/// A generic limited servo driver
pub trait ServoDriver {
    /// Set servo angle in degrees: (-45 to 45)
    fn set_angle(&mut self, angle: f32);
}

/// Delegate for setting the duty of a PWM pin
pub trait ServoDriverDelegate {
    /// Set raw duty
    fn set_duty(&mut self, duty: u8);
}

const TRAXXAS_DUTY_RANGE: [f32; 2] = [15f32, 32f32];

/// Servo driver for the Traxxas 2080 servo
#[allow(dead_code)]
pub struct Traxxas2080Servo<PIN: PinOps, TIMER> {
    pin: Pin<Output, PIN>,
    tc: *mut TIMER,
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

// Pin D3
impl Traxxas2080Servo<PD3, TC2> {
    pub fn pd3(tc2: &RefCell<TC2>, pin: Pin<Output, PD3>) -> Self {
        enable_pwm_tc2(tc2.as_ptr());
        Self {
            pin,
            tc: tc2.as_ptr(),
        }
    }
}

// Pin D3
impl ServoDriverDelegate for Traxxas2080Servo<PD3, TC2> {
    fn set_duty(&mut self, duty: u8) {
        unsafe { (*(self.tc)).ocr2b.write(|w| w.bits(duty)) }
    }
}

// Pin D11
impl Traxxas2080Servo<PB3, TC2> {
    pub fn pb3(tc2: &RefCell<TC2>, pin: Pin<Output, PB3>) -> Self {
        enable_pwm_tc2(tc2.as_ptr());
        Self {
            pin,
            tc: tc2.as_ptr(),
        }
    }
}

// Pin D11
impl ServoDriverDelegate for Traxxas2080Servo<PB3, TC2> {
    fn set_duty(&mut self, duty: u8) {
        unsafe { (*(self.tc)).ocr2a.write(|w| w.bits(duty)) }
    }
}

// Pin D9
impl Traxxas2080Servo<PB1, TC1> {
    pub fn pb1(tc1: &RefCell<TC1>, pin: Pin<Output, PB1>) -> Self {
        enable_pwm_tc1(tc1.as_ptr());
        Self {
            pin,
            tc: tc1.as_ptr(),
        }
    }
}

// Pin D9
impl ServoDriverDelegate for Traxxas2080Servo<PB1, TC1> {
    fn set_duty(&mut self, duty: u8) {
        unsafe { (*(self.tc)).ocr1a.write(|w| w.bits(duty as u16)) }
    }
}

// Pin D10
impl Traxxas2080Servo<PB2, TC1> {
    pub fn pb2(tc1: &RefCell<TC1>, pin: Pin<Output, PB2>) -> Self {
        enable_pwm_tc1(tc1.as_ptr());
        Self {
            pin,
            tc: tc1.as_ptr(),
        }
    }
}

// Pin D10
impl ServoDriverDelegate for Traxxas2080Servo<PB2, TC1> {
    fn set_duty(&mut self, duty: u8) {
        unsafe { (*(self.tc)).ocr1b.write(|w| w.bits(duty as u16)) }
    }
}

// Pin D5
impl Traxxas2080Servo<PD5, TC0> {
    pub fn pd5(tc0: &RefCell<TC0>, pin: Pin<Output, PD5>) -> Self {
        enable_pwm_tc0(tc0.as_ptr());
        Self {
            pin,
            tc: tc0.as_ptr(),
        }
    }
}

// Pin D5
impl ServoDriverDelegate for Traxxas2080Servo<PD5, TC0> {
    fn set_duty(&mut self, duty: u8) {
        unsafe { (*(self.tc)).ocr0b.write(|w| w.bits(duty)) }
    }
}

// Pin D6
impl Traxxas2080Servo<PD6, TC0> {
    pub fn pd6(tc0: &RefCell<TC0>, pin: Pin<Output, PD6>) -> Self {
        enable_pwm_tc0(tc0.as_ptr());
        Self {
            pin,
            tc: tc0.as_ptr(),
        }
    }
}

// Pin D6
impl ServoDriverDelegate for Traxxas2080Servo<PD6, TC0> {
    fn set_duty(&mut self, duty: u8) {
        unsafe { (*(self.tc)).ocr0a.write(|w| w.bits(duty)) }
    }
}

fn enable_pwm_tc1(tc1: *mut TC1) {
    unsafe {
        (*tc1)
            .tccr1a
            .write(|w| w.wgm1().bits(1).com1a().match_clear().com1b().match_clear());
        (*tc1).tccr1b.write(|w| w.cs1().prescale_1024());
    }
}

fn enable_pwm_tc2(tc2: *mut TC2) {
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

fn enable_pwm_tc0(tc0: *mut TC0) {
    if !millis_initialized() {
        unsafe {
            (*tc0).tccr0a.write(|w| {
                w.wgm0()
                    .pwm_fast()
                    .com0a()
                    .match_clear()
                    .com0b()
                    .match_clear()
            });
            (*tc0).tccr0b.write(|w| w.cs0().prescale_1024());
        }
    }
}

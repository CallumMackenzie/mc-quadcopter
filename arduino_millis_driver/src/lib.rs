#![no_std]
#![feature(abi_avr_interrupt)]

//! A crate to bring the millis() method to arduino embedded systems.

use avr_device::interrupt::Mutex as InterruptMutex;
use core::cell::Cell;
use ufmt::derive::uDebug;

const PRESCALER: u32 = 64;
const TIMER_COUNTS: u32 = 250;
const MILLIS_INCREMENT: u32 = PRESCALER * TIMER_COUNTS / 16000;

static MILLIS_COUNTER: InterruptMutex<Cell<u32>> = InterruptMutex::new(Cell::new(0));
static MILLIS_INITIALIZED: InterruptMutex<Cell<bool>> = InterruptMutex::new(Cell::new(false));

#[derive(Debug, Copy, Clone, uDebug)]
pub enum MillisErr {
    /// If this error is returned, the prescaler defaults to 1024
    InvalidPrescaler = 0,
    /// Millis has already been initialized
    AlreadyInitialized = 1,
}

fn ensure_millis_uninit() -> Result<(), MillisErr> {
    if avr_device::interrupt::free(|cs| MILLIS_INITIALIZED.borrow(cs).get()) {
        Err(MillisErr::AlreadyInitialized)
    } else {
        Ok(())
    }
}

pub fn millis_init_debug(millis: u32) -> Result<(), MillisErr> {
    ensure_millis_uninit()?;
    avr_device::interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(millis);
        MILLIS_INITIALIZED.borrow(cs).set(true);
    });
    Ok(())
}

pub fn millis_init(tc0: arduino_hal::pac::TC0) -> Result<(), MillisErr> {
    ensure_millis_uninit()?;
    tc0.tccr0a.write(|w| w.wgm0().ctc());
    tc0.ocr0a.write(|w| unsafe { w.bits(TIMER_COUNTS as u8) });
    let mut write_success = true;
    tc0.tccr0b.write(|w| match PRESCALER {
        8 => w.cs0().prescale_8(),
        64 => w.cs0().prescale_64(),
        256 => w.cs0().prescale_256(),
        1024 => w.cs0().prescale_1024(),
        _ => {
            write_success = false;
            w.cs0().prescale_1024()
        }
    });
    if !write_success {
        Err(MillisErr::InvalidPrescaler)
    } else {
        tc0.timsk0.write(|w| w.ocie0a().set_bit());
        avr_device::interrupt::free(|cs| {
            MILLIS_COUNTER.borrow(cs).set(0);
            MILLIS_INITIALIZED.borrow(cs).set(true);
        });
        unsafe { avr_device::interrupt::enable() };
        Ok(())
    }
}

#[avr_device::interrupt(atmega328p)]
fn TIMER0_COMPA() {
    avr_device::interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + MILLIS_INCREMENT);
    })
}

pub fn millis() -> u32 {
    avr_device::interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
}

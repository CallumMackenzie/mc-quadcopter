#![allow(non_camel_case_types)]
pub const ADAFRUIT1893_ADDR: u8 = 0x60;

pub struct WHOAMI;
impl WHOAMI {
    pub const ADDR: u8 = 0x0;
    pub const EXP_RESULT: u8 = 0xC4;
}

pub struct CTRL_REG1;
impl CTRL_REG1 {
    pub const ADDR: u8 = 0x26;
    pub const RESET: u8 = 0x04;
}

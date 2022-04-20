#![allow(non_snake_case, non_camel_case_types)]

// Source: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf

pub struct BitBlock {
    pub start: u8,
    pub len: u8,
}

pub struct ByteBlock {
    pub start: u8,
    pub len: u8,
}

pub const MPU_ADDR: u8 = 0x68;

pub const TEMP_OFFSET: f32 = 36.53;
pub const TEMP_SENSITIVITY: f32 = 340.0;

pub struct GYRO_CONFIG;
impl GYRO_CONFIG {
    pub const ADDR: u8 = 0x1b;
    pub const XG_ST_BIT: u8 = 7;
    pub const YG_ST_BIT: u8 = 6;
    pub const ZG_ST_BIT: u8 = 5;
    pub const GYRO_FS_SEL_BITS: BitBlock = BitBlock { start: 4, len: 2 };
    pub const FCHOICE_B_BITS: BitBlock = BitBlock { start: 0, len: 2 };
}

pub struct ACCEL_CONFIG;
impl ACCEL_CONFIG {
    pub const ADDR: u8 = 0x1c;
    pub const XA_ST_BIT: u8 = 7;
    pub const YA_ST_BIT: u8 = 6;
    pub const ZA_ST_BIT: u8 = 5;
    pub const ACCEL_FS_SEL_BITS: BitBlock = BitBlock { start: 4, len: 2 };
}

pub struct PWR_MGMT_1;
impl PWR_MGMT_1 {
    pub const ADDR: u8 = 0x6b;
    pub const RESET_BIT: u8 = 7;
    pub const SLEEP_BIT: u8 = 6;
    pub const CYCLE_BIT: u8 = 5;
    pub const GYRO_STANDBY_BIT: u8 = 4;
    pub const TEMP_DIS_BIT: u8 = 3;
    pub const CLKSEL_BITS: BitBlock = BitBlock { start: 0, len: 3 };
}

pub struct CONFIG;
impl CONFIG {
    pub const ADDR: u8 = 0x1a;
    pub const FIFO_MODE_BIT: u8 = 6;
    pub const EXT_SYNC_SET_BITS: BitBlock = BitBlock { start: 3, len: 3 };
    pub const DLPF_CFG_BITS: BitBlock = BitBlock { start: 0, len: 3 };
}

pub struct PWR_MGMT_2;
impl PWR_MGMT_2 {
    pub const ADDR: u8 = 0x6c;
    pub const LP_WAKE_CTRL_BITS: BitBlock = BitBlock { len: 2, start: 6 };
    pub const DIS_XA_BIT: u8 = 5;
    pub const DIS_YA_BIT: u8 = 4;
    pub const DIS_ZA_BIT: u8 = 3;
    pub const DIS_XG_BIT: u8 = 2;
    pub const DIS_YG_BIT: u8 = 1;
    pub const DIS_ZG_BIT: u8 = 0;
}

pub struct WHO_AM_I;
impl WHO_AM_I {
    pub const ADDR: u8 = 0x75;
    pub const WHO_AM_I_BITS: BitBlock = BitBlock { start: 0, len: 8 };
}

pub struct TEMP_OUT;
impl TEMP_OUT {
    pub const ADDR: u8 = 0x41;
    pub const BYTES: ByteBlock = ByteBlock { start: 0, len: 2 };
}

pub mod ACCEL_OUT {
    use crate::consts::ByteBlock;
    pub const ADDR: u8 = 0x3b;

    pub struct X;
    impl X {
        pub const ADDR: u8 = 0x3b;
        pub const BYTES: ByteBlock = ByteBlock { start: 0, len: 2 };
    }

    pub struct Y;
    impl Y {
        pub const ADDR: u8 = 0x3d;
        pub const BYTES: ByteBlock = ByteBlock { start: 0, len: 2 };
    }

    pub struct Z;
    impl Z {
        pub const ADDR: u8 = 0x3f;
        pub const BYTES: ByteBlock = ByteBlock { start: 0, len: 2 };
    }
}

pub mod GYRO_OUT {
    use crate::consts::ByteBlock;

    pub const ADDR: u8 = 0x43;

    pub struct X;
    impl X {
        pub const ADDR: u8 = 0x43;
        pub const BYTES: ByteBlock = ByteBlock { start: 0, len: 2 };
    }

    pub struct Y;
    impl Y {
        pub const ADDR: u8 = 0x45;
        pub const BYTES: ByteBlock = ByteBlock { start: 0, len: 2 };
    }

    pub struct Z;
    impl Z {
        pub const ADDR: u8 = 0x47;
        pub const BYTES: ByteBlock = ByteBlock { start: 0, len: 2 };
    }
}

fn read_word_2c(byte: &[u8]) -> i32 {
    let high: i32 = byte[0] as i32;
    let low: i32 = byte[1] as i32;
    let mut word: i32 = (high << 8) + low;
    if word >= 0x8000 {
        word = -((65535 - word) + 1);
    }
    word
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

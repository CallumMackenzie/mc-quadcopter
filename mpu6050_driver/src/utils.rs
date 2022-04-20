/// Reads the bytes from the array into an i32
pub fn read_word_2c(byte: &[u8]) -> i32 {
    let high: i32 = byte[0] as i32;
    let low: i32 = byte[1] as i32;
    let mut word: i32 = (high << 8) + low;
    if word >= 0x8000 {
        word = -((65535 - word) + 1);
    }
    word
}

/// Sets bits in the byte reference to data from bit_start to bit_start + length
pub fn set_bits(byte: &mut u8, bit_start: u8, length: u8, mut data: u8) {
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

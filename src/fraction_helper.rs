pub fn to_signed_u12_4_f32(value: i16) -> f32 {
    // Extract the integer part by shifting right 4 bits (keeping the sign)
    let integer_part = value >> 4;

    // Extract the fractional part by masking the lower 4 bits (as a positive value)
    let fractional_part = f32::from(value & 0xF) / 16.0;

    // Combine the integer and fractional parts
    f32::from(integer_part) + fractional_part
}

#[allow(clippy::suboptimal_flops)]
pub fn abs_f32(value: f32) -> f32 {
    if value < 0.0 {
        -value
    } else {
        value
    }
}

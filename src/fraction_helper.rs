pub fn to_signed_u12_4_to_dps(raw_value: i16) -> f32 {
    // Extract the integer part (top 12 bits)
    let integer_part = raw_value >> 4;

    // Extract the fractional part (lower 4 bits)
    let fractional_mask = 0xF; // Mask to extract the lower 4 bits (0xF = 15)
    let fractional_part = f32::from(raw_value & fractional_mask) * 0.0625; // 1 / 2^4 = 0.0625 dps

    // Combine the integer and fractional parts
    f32::from(integer_part) + fractional_part
}

pub fn to_signed_u5_11_g(raw_value: i16) -> f32 {
    // Extract the integer part (top 5 bits)
    let integer_part = raw_value >> 11;

    // Extract the fractional part (lower 11 bits)
    let fractional_mask = 0x7FF; // Mask to extract the lower 11 bits (0x7FF = 2047)
    let fractional_part = f32::from(raw_value & fractional_mask) * 0.0005; // 0.5 mg = 0.0005 g

    // Combine the integer and fractional parts
    f32::from(integer_part) + fractional_part
}

// SPDX-License-Identifier: GPL-3.0-or-later

pub const CAMERA_FEEDBACK_ID: u32 = 180;
pub const CAMERA_FEEDBACK_LEN: usize = 47;

#[allow(clippy::too_many_arguments)]
pub fn camera_feedback(
    time_us: u64,
    lat: i32,
    lon: i32,
    alt_msl: f32,
    alt_rel: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    focal_length: f32,
    image_index: u16,
    target_system: u8,
    camera_index: u8,
    flags: u8,
    completed_captures: u16,
) -> [u8; CAMERA_FEEDBACK_LEN] {
    let mut payload = [0u8; CAMERA_FEEDBACK_LEN];
    payload[0..8].copy_from_slice(&time_us.to_le_bytes());
    payload[8..12].copy_from_slice(&lat.to_le_bytes());
    payload[12..16].copy_from_slice(&lon.to_le_bytes());
    for (offset, value) in [
        (16, alt_msl),
        (20, alt_rel),
        (24, roll),
        (28, pitch),
        (32, yaw),
        (36, focal_length),
    ] {
        payload[offset..offset + 4].copy_from_slice(&value.to_bits().to_le_bytes());
    }
    payload[40..42].copy_from_slice(&image_index.to_le_bytes());
    payload[42] = target_system;
    payload[43] = camera_index;
    payload[44] = flags;
    payload[45..47].copy_from_slice(&completed_captures.to_le_bytes());
    payload
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_feedback_offsets_and_length_match_dialect() {
        let payload = camera_feedback(
            0x0102_0304_0506_0708,
            -123_456_789,
            987_654_321,
            f32::from_bits(0x3f81_0203),
            f32::from_bits(0x4081_0203),
            f32::from_bits(0x4181_0203),
            f32::from_bits(0x4281_0203),
            f32::from_bits(0x4381_0203),
            f32::from_bits(0x4481_0203),
            0x1234,
            0x56,
            0x78,
            0x9a,
            0xbcde,
        );
        assert_eq!(payload.len(), 47);
        assert_eq!(&payload[0..8], &0x0102_0304_0506_0708u64.to_le_bytes());
        assert_eq!(&payload[8..12], &(-123_456_789i32).to_le_bytes());
        assert_eq!(&payload[12..16], &987_654_321i32.to_le_bytes());
        for (offset, bits) in [
            (16, 0x3f81_0203u32),
            (20, 0x4081_0203),
            (24, 0x4181_0203),
            (28, 0x4281_0203),
            (32, 0x4381_0203),
            (36, 0x4481_0203),
        ] {
            assert_eq!(&payload[offset..offset + 4], &bits.to_le_bytes());
        }
        assert_eq!(&payload[40..], &[0x34, 0x12, 0x56, 0x78, 0x9a, 0xde, 0xbc]);
    }
}

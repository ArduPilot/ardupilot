// SPDX-License-Identifier: GPL-3.0-or-later

pub const MESSAGE_SIZE: usize = 298;
pub const PARAM_SET_ID: u32 = 23;
pub const PARAM_SET_PAYLOAD_LEN: usize = 23;
pub const PARAM_ERROR_ID: u32 = 345;
pub const PARAM_ERROR_PAYLOAD_LEN: usize = 21;
const PAYLOAD_OFFSET: usize = 15;

pub struct ParamSet<'a> {
    pub value: f32,
    pub name: &'a str,
    pub source_system: u32,
    pub source_component: u8,
}

pub fn decode_param_set(message: &[u8; MESSAGE_SIZE]) -> Option<ParamSet<'_>> {
    if !matches!(message[2], 0xfe | 0xfd)
        || (message[3] as usize) < PARAM_SET_PAYLOAD_LEN
        || (message[2] == 0xfd && message[4] & 0xf8 != 0)
    {
        return None;
    }
    let msgid = u32::from(message[12]) | u32::from(message[13]) << 8 | u32::from(message[14]) << 16;
    if msgid != PARAM_SET_ID {
        return None;
    }
    let payload = &message[PAYLOAD_OFFSET..PAYLOAD_OFFSET + PARAM_SET_PAYLOAD_LEN];
    let end = payload[6..22].iter().position(|byte| *byte == 0).unwrap_or(16);
    let name = core::str::from_utf8(&payload[6..6 + end]).ok()?;
    if name.is_empty() {
        return None;
    }
    Some(ParamSet {
        value: f32::from_bits(u32::from_le_bytes(payload[0..4].try_into().unwrap())),
        name,
        source_system: u32::from_le_bytes(message[7..11].try_into().unwrap()),
        source_component: message[11],
    })
}

pub fn param_error(component: u8, name: &str, error: u8) -> [u8; PARAM_ERROR_PAYLOAD_LEN] {
    let mut payload = [0u8; PARAM_ERROR_PAYLOAD_LEN];
    payload[0..2].copy_from_slice(&(-1i16).to_le_bytes());
    // target_system at byte two is replaced by the host from the full target.
    payload[3] = component;
    let length = core::cmp::min(name.len(), 16);
    payload[4..4 + length].copy_from_slice(&name.as_bytes()[..length]);
    payload[20] = error;
    payload
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn param_set_header_payload_offsets_and_lengths() {
        let mut message = [0u8; MESSAGE_SIZE];
        message[2] = 0xfd;
        message[3] = PARAM_SET_PAYLOAD_LEN as u8;
        message[7..11].copy_from_slice(&0x89ab_cdefu32.to_le_bytes());
        message[11] = 250;
        message[12..15].copy_from_slice(&[23, 0, 0]);
        let payload = &mut message[PAYLOAD_OFFSET..PAYLOAD_OFFSET + PARAM_SET_PAYLOAD_LEN];
        payload[0..4].copy_from_slice(&78.25f32.to_bits().to_le_bytes());
        payload[4] = 1;
        payload[5] = 1;
        payload[6..18].copy_from_slice(b"DISARM_DELAY");
        payload[22] = 9;
        let decoded = decode_param_set(&message).unwrap();
        assert_eq!(decoded.value.to_bits(), 78.25f32.to_bits());
        assert_eq!(decoded.name, "DISARM_DELAY");
        assert_eq!(decoded.source_system, 0x89ab_cdef);
        assert_eq!(decoded.source_component, 250);
    }

    #[test]
    fn param_error_offsets_and_length_match_dialect() {
        let payload = param_error(250, "DISARM_DELAY", 3);
        assert_eq!(payload.len(), 21);
        assert_eq!(&payload[0..2], &[0xff, 0xff]);
        assert_eq!(payload[2], 0);
        assert_eq!(payload[3], 250);
        assert_eq!(&payload[4..16], b"DISARM_DELAY");
        assert_eq!(&payload[16..20], &[0; 4]);
        assert_eq!(payload[20], 3);
    }

    #[test]
    fn malformed_or_short_messages_are_rejected() {
        let mut message = [0u8; MESSAGE_SIZE];
        assert!(decode_param_set(&message).is_none());
        message[2] = 0xfd;
        message[3] = 22;
        message[12] = 23;
        assert!(decode_param_set(&message).is_none());
        message[3] = 23;
        message[4] = 0x80;
        assert!(decode_param_set(&message).is_none());
    }
}

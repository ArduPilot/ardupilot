// SPDX-License-Identifier: GPL-3.0-or-later
//! MAVLink framing helpers and the Skybrush custom packet formats.

pub const MESSAGE_SIZE: usize = 298;
const PAYLOAD_OFFSET: usize = 15;

pub const MSG_COMMAND_INT: u32 = 75;
pub const MSG_COMMAND_ACK: u32 = 77;
pub const MSG_DATA16: u32 = 169;
pub const MSG_DATA32: u32 = 170;
pub const MSG_DATA64: u32 = 171;
pub const MSG_DATA96: u32 = 172;
pub const MSG_LED_CONTROL: u32 = 186;
pub const MSG_TUNNEL: u32 = 385;

pub const MAV_CMD_USER_1: u16 = 31010;
pub const MAV_CMD_USER_2: u16 = 31011;

pub const MAV_RESULT_ACCEPTED: u8 = 0;
pub const MAV_RESULT_DENIED: u8 = 2;
pub const MAV_RESULT_UNSUPPORTED: u8 = 3;
pub const MAV_RESULT_FAILED: u8 = 4;

// DATA* packet types
pub const TYPE_STATUS: u8 = 0x5b;
pub const TYPE_GCS_TO_DRONE: u8 = 0x5c;
pub const TYPE_DRONE_TO_GCS: u8 = 0x5d;

// inner packet types for TYPE_GCS_TO_DRONE
pub const START_CONFIG: u8 = 1;
pub const DEPRECATED_CRTL_TRIGGER: u8 = 2;
pub const SIMPLE_GEOFENCE_SETUP: u8 = 3;
pub const ACKNOWLEDGMENT: u8 = 4;
pub const TIME_AXIS_CONFIG: u8 = 5;

pub const TUNNEL_PIXEL_GRID: u16 = 42421;

/// A received message: id, source and the payload zero-extended to its full size.
pub struct Received {
    pub msgid: u32,
    pub sysid: u32,
    pub compid: u8,
    pub channel: u8,
    payload: [u8; 255],
}

impl Received {
    pub fn parse(bytes: &[u8; MESSAGE_SIZE], channel: u8) -> Option<Self> {
        if !matches!(bytes[2], 0xfe | 0xfd) {
            return None;
        }
        let len = bytes[3] as usize;
        let msgid = u32::from(bytes[12]) | u32::from(bytes[13]) << 8 | u32::from(bytes[14]) << 16;
        let mut payload = [0u8; 255];
        // MAVLink2 truncates trailing zeros: anything past len reads as zero
        payload[..len].copy_from_slice(&bytes[PAYLOAD_OFFSET..PAYLOAD_OFFSET + len]);
        Some(Self {
            msgid,
            sysid: u32::from_le_bytes(bytes[7..11].try_into().unwrap()),
            compid: bytes[11],
            channel,
            payload,
        })
    }

    pub fn payload(&self) -> &[u8; 255] {
        &self.payload
    }

    /// DATA16/32/64/96 as (type, data).
    pub fn data_packet(&self) -> Option<(u8, &[u8])> {
        let cap = match self.msgid {
            MSG_DATA16 => 16,
            MSG_DATA32 => 32,
            MSG_DATA64 => 64,
            MSG_DATA96 => 96,
            _ => return None,
        };
        let len = (self.payload[1] as usize).min(cap);
        Some((self.payload[0], &self.payload[2..2 + len]))
    }

    pub fn command_int(&self) -> Option<CommandInt> {
        if self.msgid != MSG_COMMAND_INT {
            return None;
        }
        let p = &self.payload;
        Some(CommandInt {
            param1: f32_at(p, 0),
            param2: f32_at(p, 4),
            param3: f32_at(p, 8),
            param4: f32_at(p, 12),
            x: i32_at(p, 16),
            y: i32_at(p, 20),
            z: f32_at(p, 24),
            command: u16_at(p, 28),
            target_system: p[30],
            target_component: p[31],
            frame: p[32],
        })
    }
}

#[derive(Clone, Copy)]
pub struct CommandInt {
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub x: i32,
    pub y: i32,
    pub z: f32,
    pub command: u16,
    pub target_system: u8,
    pub target_component: u8,
    pub frame: u8,
}

pub fn u16_at(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
pub fn i16_at(b: &[u8], o: usize) -> i16 {
    u16_at(b, o) as i16
}
pub fn u32_at(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(b[o..o + 4].try_into().unwrap())
}
pub fn i32_at(b: &[u8], o: usize) -> i32 {
    u32_at(b, o) as i32
}
pub fn u64_at(b: &[u8], o: usize) -> u64 {
    u64::from_le_bytes(b[o..o + 8].try_into().unwrap())
}
pub fn f32_at(b: &[u8], o: usize) -> f32 {
    f32::from_bits(u32_at(b, o))
}

/// Read a little-endian field from a possibly short slice, treating missing bytes as zero.
pub fn padded<const N: usize>(b: &[u8], o: usize) -> [u8; N] {
    let mut out = [0u8; N];
    for (i, byte) in out.iter_mut().enumerate() {
        if let Some(v) = b.get(o + i) {
            *byte = *v;
        }
    }
    out
}

/// START_CONFIG body (after the inner type byte).
pub struct StartConfig {
    pub start_time: i32,
    pub authorization: u8,
    pub countdown_msec: Option<i32>,
    pub msec_offset: Option<u16>,
}

pub fn decode_start_config(d: &[u8]) -> Option<StartConfig> {
    if d.len() < 5 {
        return None;
    }
    Some(StartConfig {
        start_time: i32_at(d, 0),
        authorization: d[4],
        countdown_msec: (d.len() >= 9).then(|| i32_at(d, 5)),
        msec_offset: (d.len() >= 11).then(|| u16_at(d, 9)),
    })
}

pub struct GeofenceSetup<'a> {
    pub max_altitude_dm: u16,
    pub radius_dm: u16,
    pub ack_token: u16,
    pub flags: u8,
    points: &'a [u8],
    pub num_points: usize,
}

impl GeofenceSetup<'_> {
    /// (x, y) of point i in decimetres, show frame.
    pub fn point(&self, i: usize) -> (i16, i16) {
        (i16_at(self.points, i * 4), i16_at(self.points, i * 4 + 2))
    }
}

pub fn decode_geofence_setup(d: &[u8]) -> Option<GeofenceSetup<'_>> {
    if d.len() < 11 {
        return None;
    }
    let num_points = d[0] as usize;
    let points = &d[11..];
    if points.len() < num_points * 4 {
        return None;
    }
    Some(GeofenceSetup {
        max_altitude_dm: u16_at(d, 1),
        radius_dm: u16_at(d, 3),
        ack_token: u16_at(d, 5),
        flags: d[7],
        points,
        num_points,
    })
}

/// DATA16/DATA96 payload: type, len, data padded to the message size.
pub fn data_payload<const N: usize>(kind: u8, data: &[u8]) -> [u8; N] {
    let mut out = [0u8; N];
    out[0] = kind;
    out[1] = data.len() as u8;
    out[2..2 + data.len()].copy_from_slice(data);
    out
}

/// Acknowledgment sent as DATA16 0x5D, matching the fork's 6-byte struct.
pub fn ack_data(token: u16, result: u8) -> [u8; 18] {
    let mut d = [0u8; 6];
    d[0] = ACKNOWLEDGMENT;
    d[1..3].copy_from_slice(&token.to_le_bytes());
    d[3] = result;
    data_payload::<18>(TYPE_DRONE_TO_GCS, &d)
}

pub fn command_ack(command: u16, result: u8, target_system: u8, target_component: u8) -> [u8; 10] {
    let mut p = [0u8; 10];
    p[0..2].copy_from_slice(&command.to_le_bytes());
    p[2] = result;
    p[8] = target_system;
    p[9] = target_component;
    p
}

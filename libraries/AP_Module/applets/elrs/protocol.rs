// SPDX-License-Identifier: GPL-3.0-or-later
// ExpressLRS wire protocol; see README.md for upstream provenance.
pub const OTA_VERSION: u8 = 4;
pub const STANDARD_SIZE: usize = 8;
pub const FULL_SIZE: usize = 13;

pub fn uid_seed(uid: &[u8; 6]) -> u32 {
    u32::from_be_bytes([uid[2], uid[3], uid[4], uid[5] ^ OTA_VERSION])
}
pub fn crc_initializer(uid: &[u8; 6]) -> u16 {
    u16::from_be_bytes([uid[4], uid[5]]) ^ (u16::from(OTA_VERSION) << 8)
}
pub fn uid_is_bound(uid: &[u8; 6]) -> bool {
    uid[2..].iter().any(|&x| x != 0)
}
fn valid_size(size: usize) -> bool {
    size == STANDARD_SIZE || size == FULL_SIZE
}
fn packet_crc(packet: &[u8], mut crc: u16) -> u16 {
    let standard = packet.len() == STANDARD_SIZE;
    let (bits, poly, length) = if standard {
        (14, 0x2e57, 7)
    } else {
        (16, 0x3d65, 11)
    };
    for (i, &value) in packet[..length].iter().enumerate() {
        let byte = if standard && i == 0 { value & 3 } else { value };
        crc ^= u16::from(byte) << (bits - 8);
        for _ in 0..8 {
            let high = crc & (1 << (bits - 1)) != 0;
            crc = (crc << 1) ^ if high { poly } else { 0 };
        }
    }
    crc & if standard { 0x3fff } else { 0xffff }
}
pub fn validate_packet(packet: &[u8], init: u16, nonce: u8) -> bool {
    if !valid_size(packet.len()) || packet[0] & 3 > 2 {
        return false;
    }
    let received = if packet.len() == STANDARD_SIZE {
        (u16::from(packet[0] >> 2) << 8) | u16::from(packet[7])
    } else {
        u16::from_le_bytes([packet[11], packet[12]])
    };
    let nonce = if packet[0] & 3 == 2 { 0 } else { nonce };
    received == packet_crc(packet, init ^ u16::from(nonce))
}
pub fn telemetry_denominator(ratio: u8) -> u8 {
    if (1..=7).contains(&ratio) {
        1 << (8 - ratio)
    } else {
        1
    }
}
pub fn make_link_stats(
    packet: &mut [u8],
    init: u16,
    nonce: u8,
    rssi: i8,
    snr: i8,
    lq: u8,
    ack: bool,
) -> bool {
    if !valid_size(packet.len()) {
        return false;
    }
    packet.fill(0);
    let full = packet.len() == FULL_SIZE;
    let offset = if full { 1 } else { 2 };
    packet[if full { 0 } else { 1 }] = if ack {
        if full {
            4
        } else {
            0x80
        }
    } else {
        0
    };
    packet[offset] = (-i16::from(rssi)).clamp(0, 127) as u8;
    packet[offset + 1] = 0x80;
    packet[offset + 2] = lq.min(100);
    packet[offset + 3] = snr as u8;
    let crc = packet_crc(packet, init ^ u16::from(nonce));
    if full {
        packet[11..13].copy_from_slice(&crc.to_le_bytes());
    } else {
        packet[0] = (packet[0] & 3) | ((crc >> 8) as u8) << 2;
        packet[7] = crc as u8;
    }
    true
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sync {
    pub fhss_index: u8,
    pub nonce: u8,
    pub rate: u8,
    pub switch_mode: u8,
    pub telemetry_ratio: u8,
    pub gemini: bool,
    pub protocol: u8,
    pub uid4: u8,
    pub uid5: u8,
}
pub fn parse_sync(packet: &[u8]) -> Option<Sync> {
    if !valid_size(packet.len()) || packet[0] & 3 != 2 {
        return None;
    }
    Some(Sync {
        fhss_index: packet[1],
        nonce: packet[2],
        rate: packet[3],
        switch_mode: packet[4] & 1,
        telemetry_ratio: (packet[4] >> 1) & 7,
        gemini: packet[4] & 0x10 != 0,
        protocol: (packet[4] >> 5) & 3,
        uid4: packet[5],
        uid5: packet[6],
    })
}
pub fn parse_binding_uid(packet: &[u8]) -> Option<[u8; 6]> {
    if packet.len() != STANDARD_SIZE
        || !validate_packet(packet, u16::from(OTA_VERSION), 0)
        || packet[0] & 3 != 1
        || packet[1] & 0x7f != 1
        || packet[2] != 9
    {
        return None;
    }
    let uid = [0, 0, packet[3], packet[4], packet[5], packet[6]];
    uid_is_bound(&uid).then_some(uid)
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Channels {
    pub crsf: [u16; 16],
    pub updated_mask: u16,
    pub armed: bool,
    pub telemetry_ack: bool,
    pub uplink_power: u8,
}
impl Channels {
    pub const fn new() -> Self {
        Self {
            crsf: [0; 16],
            updated_mask: 0,
            armed: false,
            telemetry_ack: false,
            uplink_power: 0,
        }
    }
}
fn switch_value(value: u8, maximum: u16) -> u16 {
    (u32::from(value) * 1601 / u32::from(maximum) + 191) as u16
}
fn switch_3bit(value: u8) -> u16 {
    match value {
        0 => 191,
        5 => 1792,
        6 | 7 => 992,
        _ => u16::from(value) * 240 + 391,
    }
}
pub fn decode_channels(
    packet: &[u8],
    init: u16,
    nonce: u8,
    mode: u8,
    channels: &mut Channels,
) -> bool {
    if !validate_packet(packet, init, nonce) || packet[0] & 3 != 0 || mode > 1 {
        return false;
    }
    if packet.len() == FULL_SIZE {
        let high = packet[0] & 0x40 != 0;
        let low = if mode == 1 && high { 8 } else { 0 };
        let upper = if mode == 1 {
            low + 4
        } else if high {
            8
        } else {
            4
        };
        channels.updated_mask = 0;
        for group in 0..2 {
            let mut packed = 0u64;
            for i in 0..5 {
                packed |= u64::from(packet[1 + group * 5 + i]) << (8 * i);
            }
            for i in 0..4 {
                let channel = if group == 0 { low } else { upper } + i;
                channels.crsf[channel] = (((packed >> (10 * i)) & 1023) << 1) as u16;
                channels.updated_mask |= 1 << channel;
            }
        }
        channels.armed = packet[0] & 0x80 != 0;
        channels.telemetry_ack = packet[0] & 4 != 0;
        channels.uplink_power = ((packet[0] >> 3) & 7) + 1;
        if mode == 0 {
            channels.crsf[13] = if channels.armed { 1792 } else { 191 };
            channels.updated_mask |= 1 << 13;
        }
        return true;
    }
    let mut packed = 0u64;
    for i in 0..5 {
        packed |= u64::from(packet[1 + i]) << (8 * i);
    }
    for i in 0..4 {
        let raw = ((packed >> (10 * i)) & 1023) as u32;
        channels.crsf[i] = ((raw * 1639 * 2 / 1023 + 345) / 2) as u16;
    }
    channels.armed = packet[6] & 0x80 != 0;
    channels.crsf[4] = if channels.armed { 1792 } else { 191 };
    channels.crsf[13] = channels.crsf[4];
    channels.updated_mask = 0x201f;
    channels.telemetry_ack = packet[6] & 0x40 != 0;
    let switches = packet[6] & 0x7f;
    let index;
    if mode == 1 {
        let selector = (switches >> 3) & 7;
        index = if selector >= 6 {
            11
        } else {
            usize::from(selector) + 5
        };
        channels.crsf[index] = if selector >= 6 {
            switch_value(switches & 15, 15)
        } else {
            switch_3bit(switches & 7)
        };
    } else {
        let selector = ((nonce & 7) + ((nonce >> 3) & 1)) % 8;
        if selector == 7 {
            channels.uplink_power = switches & 63;
            return true;
        }
        index = usize::from(selector) + 5;
        channels.crsf[index] = switch_value(switches & 63, 63);
    }
    channels.updated_mask |= 1 << index;
    true
}
#[derive(Clone)]
pub struct Fhss {
    sequence: [u8; 240],
    index: u8,
}
impl Fhss {
    pub fn new(mut seed: u32) -> Self {
        let mut sequence = [0; 240];
        for (i, value) in sequence.iter_mut().enumerate() {
            *value = match i % 80 {
                0 => 40,
                40 => 0,
                x => x as u8,
            };
        }
        for i in 0..240 {
            if i % 80 == 0 {
                continue;
            }
            seed = seed.wrapping_mul(214013).wrapping_add(2531011) & 0x7fffffff;
            let other = (i / 80) * 80 + ((seed >> 16) % 79) as usize + 1;
            sequence.swap(i, other);
        }
        Self { sequence, index: 0 }
    }
    pub fn index(&self) -> u8 {
        self.index
    }
    pub fn set_index(&mut self, index: u8) {
        self.index = index % 240;
    }
    pub fn channel(&self) -> u8 {
        self.sequence[usize::from(self.index)]
    }
    pub fn frequency(&self) -> u32 {
        Self::channel_frequency(self.channel())
    }
    pub fn next_frequency(&mut self) -> u32 {
        self.index = (self.index + 1) % 240;
        self.frequency()
    }
    pub fn channel_frequency(channel: u8) -> u32 {
        const START: u32 = (2_400_400_000u64 * 262144 / 52_000_000) as u32;
        const STOP: u32 = (2_479_400_000u64 * 262144 / 52_000_000) as u32;
        const SPREAD: u32 = (STOP - START) * 256 / 79;
        START + SPREAD * u32::from(channel) / 256
    }
}

// DATA and link-statistics fragments use the same CRC and bounded OTA layouts.
pub fn make_downlink(
    packet: &mut [u8],
    init: u16,
    nonce: u8,
    link_stats: bool,
    index: u8,
    payload: &[u8; 10],
    ack: bool,
    rssi: i8,
    snr: i8,
    lq: u8,
) -> bool {
    if !make_link_stats(packet, init, nonce, rssi, snr, lq, ack) {
        return false;
    }
    let full = packet.len() == FULL_SIZE;
    packet[0] = if link_stats { 0 } else { 1 };
    if full {
        packet[0] |= (index << 3) | if ack { 4 } else { 0 };
    } else {
        packet[1] = index | if ack { 0x80 } else { 0 };
    }
    let capacity = (if full { 10 } else { 5 }) - if link_stats { 4 } else { 0 };
    let offset = (if full { 1 } else { 2 }) + if link_stats { 4 } else { 0 };
    packet[offset..offset + capacity].copy_from_slice(&payload[..capacity]);
    let crc = packet_crc(packet, init ^ u16::from(nonce));
    if full {
        packet[11..13].copy_from_slice(&crc.to_le_bytes());
    } else {
        packet[0] = (packet[0] & 3) | ((crc >> 8) as u8) << 2;
        packet[7] = crc as u8;
    }
    true
}

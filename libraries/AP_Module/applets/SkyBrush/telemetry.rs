// SPDX-License-Identifier: GPL-3.0-or-later
//! Skybrush status packets (DATA16 basic / DATA96 extended, type 0x5B).

pub const BASIC_LEN: usize = 14;
pub const EXTENDED_LEN: usize = 48;

/// MSG ids for the base streams the fork configures per telemetry profile.
pub const MSG_GLOBAL_POSITION_INT: u32 = 33;
pub const MSG_SYS_STATUS: u32 = 1;
pub const MSG_GPS_RAW_INT: u32 = 24;

#[derive(Default, Clone, Copy)]
pub struct Basic {
    pub start_time: i32,
    pub led_rgb565: u16,
    pub show_ok: bool,
    pub start_scheduled: bool,
    pub origin_set: bool,
    pub orientation_set: bool,
    pub fence_enabled: bool,
    pub authorized: bool,
    pub gps_time_bad: bool,
    pub fence_breached: bool,
    pub preflight_failures: u8,
    pub stage: u8,
    pub gps_status: u8,
    pub num_sats: u8,
    pub boot_count: u8,
    pub auth_scope: u8,
    pub esc_error: bool,
    pub off_position: bool,
    pub elapsed_sec: f32,
    pub rtcm_counts: [i16; 2],
}

#[derive(Default, Clone, Copy)]
pub struct Extended {
    pub lat: i32,
    pub lng: i32,
    pub alt_mm: i32,
    pub relative_alt_mm: i32,
    pub vel_cms: [i16; 3],
    pub hdg_cd: u16,
    pub hdop: u16,
    pub vdop: u16,
}

fn bit(cond: bool, n: u8) -> u8 {
    (cond as u8) << n
}

fn floor_i16(v: f32) -> i16 {
    if v > 32767.0 {
        32767
    } else if v <= -32768.0 {
        -32768
    } else {
        let t = v as i32;
        // truncation rounds toward zero; floor for negatives
        (if (t as f32) > v { t - 1 } else { t }) as i16
    }
}

pub fn encode_basic(s: &Basic, out: &mut [u8]) {
    out[0..4].copy_from_slice(&s.start_time.to_le_bytes());
    out[4..6].copy_from_slice(&s.led_rgb565.to_le_bytes());
    out[6] = bit(s.show_ok, 7)
        | bit(s.start_scheduled, 6)
        | bit(s.origin_set, 5)
        | bit(s.orientation_set, 4)
        | bit(s.fence_enabled, 3)
        | bit(s.authorized, 2)
        | bit(s.gps_time_bad, 1)
        | bit(s.fence_breached, 0);
    out[7] = (s.preflight_failures & 0xf0) | (s.stage & 0x0f);
    out[8] = s.gps_status.min(7) | (s.num_sats.min(31) << 3);
    out[9] = (s.boot_count & 0x03)
        | ((s.auth_scope & 0x03) << 2)
        | bit(s.esc_error, 6)
        | bit(s.off_position, 7);
    out[10..12].copy_from_slice(&floor_i16(s.elapsed_sec).to_le_bytes());
    for (i, c) in s.rtcm_counts.iter().enumerate() {
        out[12 + i] = if *c < 0 { 0 } else { (*c).min(254) as u8 + 1 };
    }
}

pub fn encode_extended(s: &Basic, e: &Extended, out: &mut [u8]) {
    encode_basic(s, out);
    out[14..18].copy_from_slice(&e.lat.to_le_bytes());
    out[18..22].copy_from_slice(&e.lng.to_le_bytes());
    out[22..26].copy_from_slice(&e.alt_mm.to_le_bytes());
    out[26..30].copy_from_slice(&e.relative_alt_mm.to_le_bytes());
    for i in 0..3 {
        out[30 + 2 * i..32 + 2 * i].copy_from_slice(&e.vel_cms[i].to_le_bytes());
    }
    out[36..38].copy_from_slice(&e.hdg_cd.to_le_bytes());
    out[38..40].copy_from_slice(&e.hdop.to_le_bytes());
    out[40..42].copy_from_slice(&e.vdop.to_le_bytes());
    // show_id unknown (0), trajectory index unknown
    out[42..46].copy_from_slice(&0u32.to_le_bytes());
    out[46..48].copy_from_slice(&0xFFFFu16.to_le_bytes());
}

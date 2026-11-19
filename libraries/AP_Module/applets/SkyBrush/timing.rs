// SPDX-License-Identifier: GPL-3.0-or-later
//! Show clock: GPS-time or countdown based start time, as in the Skybrush fork.

pub const GPS_WEEK_SEC: i64 = 604800;
const MSEC_PER_WEEK: u64 = 604800000;
const GPS_LEAPSECONDS_MSEC: u64 = 18000;
// AP_GPS UNIX_OFFSET_MSEC
const UNIX_OFFSET_MSEC: u64 = 17000 * 86400 + 52 * 10 * MSEC_PER_WEEK - GPS_LEAPSECONDS_MSEC;

/// Elapsed time used when no start time is set (one day before the start).
pub const UNSET_MSEC: i32 = -86400000;

/// AP_GPS::istate_time_to_epoch_ms
pub fn gps_to_unix_ms(week: u16, tow_ms: u32) -> u64 {
    UNIX_OFFSET_MSEC + week as u64 * MSEC_PER_WEEK + tow_ms as u64
}

/// UNIX epoch ms to (GPS week, time of week ms)
pub fn unix_ms_to_week_tow_ms(unix_ms: u64) -> (u16, u32) {
    let since = unix_ms.saturating_sub(UNIX_OFFSET_MSEC);
    ((since / MSEC_PER_WEEK).min(u16::MAX as u64) as u16, (since % MSEC_PER_WEEK) as u32)
}

/// UNIX epoch ms to (GPS time of week s, ms offset), as the fork's
/// TIME_AXIS_CONFIG handler; tow is -1 for 0 (unset).
pub fn unix_ms_to_tow(unix_ms: u64) -> (i32, i32) {
    if unix_ms == 0 {
        return (-1, 0);
    }
    let since = unix_ms.wrapping_sub(UNIX_OFFSET_MSEC);
    (((since / 1000) % GPS_WEEK_SEC as u64) as i32, (since % 1000) as i32)
}

/// Pick the GPS week that puts a time-of-week start nearest to now.
pub fn resolve_start_unix_usec(start_tow_ms: i64, now_week: u16, now_tow_ms: u32) -> u64 {
    let half_week = GPS_WEEK_SEC * 1000 / 2;
    let diff = start_tow_ms - now_tow_ms as i64;
    let mut week = now_week as i32;
    if diff > half_week {
        week -= 1;
    } else if diff < -half_week {
        week += 1;
    }
    gps_to_unix_ms(week.clamp(0, u16::MAX as i32) as u16, start_tow_ms as u32) * 1000
}

#[derive(Default)]
pub struct Clock {
    last_gps_usec: u64,
    /// UNIX epoch start time in GPS mode, 0 when unset
    pub start_unix_usec: u64,
    /// micros64 start time in countdown mode, 0 when unset
    pub start_internal_usec: u64,
    pub use_gps: bool,
}

impl Clock {
    /// Monotonic GPS time: the fork ignores backward steps of the reported time.
    pub fn gps_now_usec(&mut self, raw: u64) -> u64 {
        if raw > self.last_gps_usec {
            self.last_gps_usec = raw;
        }
        self.last_gps_usec
    }

    /// Signed time since start, None if no start time is set.
    pub fn elapsed_usec(&mut self, gps_raw_usec: u64, micros64: u64) -> Option<i64> {
        let (now, reference) = if self.use_gps {
            (self.gps_now_usec(gps_raw_usec), self.start_unix_usec)
        } else {
            (micros64, self.start_internal_usec)
        };
        if reference == 0 {
            return None;
        }
        Some(now as i64 - reference as i64)
    }
}

/// Clamp to ±1 day in milliseconds, as get_elapsed_time_since_start_msec().
pub fn elapsed_msec(elapsed_usec: Option<i64>) -> i32 {
    match elapsed_usec {
        None => UNSET_MSEC,
        Some(us) if us <= -86400000000 => -86400000,
        Some(us) if us >= 86400000000 => 86400000,
        Some(us) => (us / 1000) as i32,
    }
}

pub fn elapsed_sec(elapsed_usec: Option<i64>) -> f32 {
    match elapsed_usec {
        None => -86400.0,
        Some(us) => (us / 1000) as f32 / 1000.0,
    }
}

/// Status packet start time: bits 0-19 GPS ToW seconds, bits 20-29 milliseconds.
pub fn encode_start_time(tow_sec: i32, msec: i32) -> i32 {
    if tow_sec < 0 {
        -1
    } else {
        (tow_sec & 0xFFFFF) | ((msec & 0x3FF) << 20)
    }
}

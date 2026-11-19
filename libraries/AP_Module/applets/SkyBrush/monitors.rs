// SPDX-License-Identifier: GPL-3.0-or-later
//! Status inputs derived from host counters: RTCM message rates, ESC error
//! rates and compass calibration results, as reported by the Skybrush fork.

use ap_rust_sdk::{Context, Error, NotifyEvents, Result};

/// Capability bit GCSes use to detect Skybrush firmware
pub const SKYBRUSH_CAPABILITY: u32 = 0x400_0000;

const RTCM_WINDOW_SEC: u32 = 5;

/// Messages received in the last 5 s in one-second buckets, as the fork's
/// MessageCounter; fed from the host's running per-channel total.
#[derive(Default)]
pub struct RtcmRate {
    last_total: u32,
    buckets: [(u32, u32); RTCM_WINDOW_SEC as usize],
}

impl RtcmRate {
    pub fn update(&mut self, total: u32, now_ms: u32) {
        let added = total.wrapping_sub(self.last_total);
        self.last_total = total;
        let sec = now_ms / 1000;
        let bucket = &mut self.buckets[(sec % RTCM_WINDOW_SEC) as usize];
        if bucket.0 != sec {
            *bucket = (sec, 0);
        }
        bucket.1 = bucket.1.saturating_add(added);
    }

    /// Recent message count, -1 if none were ever received
    pub fn count(&self, now_ms: u32) -> i16 {
        if self.last_total == 0 {
            return -1;
        }
        let sec = now_ms / 1000;
        let sum: u32 = self
            .buckets
            .iter()
            .filter(|(s, _)| sec.wrapping_sub(*s) < RTCM_WINDOW_SEC)
            .map(|(_, n)| *n)
            .sum();
        sum.min(i16::MAX as u32) as i16
    }
}

const MAX_ESCS: usize = 32;

/// Last reported RPM error rate per ESC. Stale ESCs keep their last value,
/// as the fork treats an ESC that stops reporting as still erroring.
pub struct EscErrors {
    rates: [Option<f32>; MAX_ESCS],
}

impl Default for EscErrors {
    fn default() -> Self {
        Self { rates: [None; MAX_ESCS] }
    }
}

impl EscErrors {
    pub fn update(&mut self, c: &Context<'_>) -> Result {
        for (i, rate) in self.rates.iter_mut().enumerate() {
            match c.esc_telem().get_raw_rpm_and_error_rate(i as u8) {
                Ok(Some((_, e))) => *rate = Some(e),
                Ok(None) => {}
                // no ESC telemetry in this firmware
                Err(Error::Unavailable) => return Ok(()),
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    /// True if any ESC's error rate exceeds the threshold in percent; negative disables
    pub fn high(&self, threshold_pct: f32) -> bool {
        threshold_pct >= 0.0 && self.rates.iter().flatten().any(|e| *e > threshold_pct)
    }
}

/// Compass calibration results seen since the last poll.
#[derive(Default)]
pub struct CalEvents {
    last: Option<u64>,
}

pub struct CalResult {
    pub failed: bool,
    pub saved: bool,
}

impl CalEvents {
    pub fn update(&mut self, counts: u64) -> CalResult {
        let changed = |shift: u32| match self.last {
            Some(last) => (counts >> shift) as u8 != (last >> shift) as u8,
            None => false,
        };
        let r = CalResult {
            failed: changed(NotifyEvents::COMPASS_CAL_FAILED),
            saved: changed(NotifyEvents::COMPASS_CAL_SAVED),
        };
        self.last = Some(counts);
        r
    }
}

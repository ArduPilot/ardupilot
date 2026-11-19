// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush yaw_control/yaw_control.c (Copyright CollMot Robotics Ltd).
//! Yaw control block (type 5): flags u8 (bit0 auto yaw), yaw offset i16
//! (0.1 deg), then {duration u16 ms, yaw change i16 (0.1 deg)} setpoints.

use alloc::vec::Vec;

use super::binary::{read_i16, read_u16, ShowFile, BLOCK_YAW_CONTROL};
use super::fmath::roundf;
use super::{Error, Result};

const HEADER: usize = 3;
const DELTA: usize = 4;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct YawControl {
    buf: Vec<u8>,
    pub auto_yaw: bool,
    pub yaw_offset_ddeg: i16,
}

impl YawControl {
    pub fn from_bytes(body: &[u8]) -> Result<Self> {
        if body.len() < HEADER || (body.len() - HEADER) % DELTA != 0 {
            return Err(Error::Corrupted);
        }
        let mut off = 1;
        let yaw_offset_ddeg = read_i16(body, &mut off);
        Ok(YawControl { buf: body.to_vec(), auto_yaw: body[0] & 1 != 0, yaw_offset_ddeg })
    }

    pub fn from_show(file: &ShowFile) -> Result<Self> {
        Self::from_bytes(file.find_first(BLOCK_YAW_CONTROL)?)
    }

    pub fn constant(yaw_deg: f32) -> Self {
        let ddeg = roundf(yaw_deg * 10.0).clamp(-32768.0, 32767.0) as i16;
        let b = ddeg.to_le_bytes();
        YawControl { buf: alloc::vec![0, b[0], b[1]], auto_yaw: false, yaw_offset_ddeg: ddeg }
    }

    pub fn num_deltas(&self) -> usize {
        (self.buf.len() - HEADER) / DELTA
    }

    pub fn is_empty(&self) -> bool {
        self.num_deltas() == 0
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct YawSetpoint {
    pub start_time_msec: u32,
    pub end_time_msec: u32,
    pub duration_msec: u32,
    pub start_time_sec: f32,
    pub end_time_sec: f32,
    pub duration_sec: f32,
    pub start_yaw_ddeg: i32,
    pub end_yaw_ddeg: i32,
    pub yaw_change_ddeg: i32,
    pub start_yaw_deg: f32,
    pub end_yaw_deg: f32,
    pub yaw_change_deg: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct YawPlayer {
    start: usize,
    length: usize,
    pub setpoint: YawSetpoint,
}

impl YawPlayer {
    pub fn new(ctrl: &YawControl) -> Self {
        let mut p = YawPlayer::default();
        p.rewind(ctrl);
        p
    }

    pub fn rewind(&mut self, ctrl: &YawControl) {
        self.build(ctrl, HEADER, 0, ctrl.yaw_offset_ddeg as i32);
    }

    fn build(&mut self, ctrl: &YawControl, mut off: usize, start_time_msec: u32, start_yaw_ddeg: i32) {
        let mut s = YawSetpoint {
            start_time_msec,
            start_time_sec: start_time_msec as f32 / 1000.0,
            start_yaw_ddeg,
            start_yaw_deg: start_yaw_ddeg as f32 / 10.0,
            ..Default::default()
        };
        self.start = off;
        self.length = 0;
        if off >= ctrl.buf.len() {
            s.duration_msec = u32::MAX - start_time_msec;
            s.duration_sec = f32::INFINITY;
            s.end_time_msec = u32::MAX;
            s.end_time_sec = f32::INFINITY;
            s.end_yaw_deg = s.start_yaw_deg;
            s.end_yaw_ddeg = s.start_yaw_ddeg;
        } else {
            s.duration_msec = read_u16(&ctrl.buf, &mut off) as u32;
            s.duration_sec = s.duration_msec as f32 / 1000.0;
            s.end_time_msec = start_time_msec.wrapping_add(s.duration_msec);
            s.end_time_sec = s.end_time_msec as f32 / 1000.0;
            s.yaw_change_ddeg = read_i16(&ctrl.buf, &mut off) as i32;
            s.yaw_change_deg = s.yaw_change_ddeg as f32 / 10.0;
            s.end_yaw_ddeg = s.start_yaw_ddeg + s.yaw_change_ddeg;
            s.end_yaw_deg = s.end_yaw_ddeg as f32 / 10.0;
            self.length = off - self.start;
        }
        self.setpoint = s;
    }

    fn build_next(&mut self, ctrl: &YawControl) {
        let s = self.setpoint;
        self.build(ctrl, self.start + self.length, s.end_time_msec, s.end_yaw_ddeg);
    }

    pub fn has_more_setpoints(&self) -> bool {
        self.length > 0
    }

    fn seek(&mut self, ctrl: &YawControl, t: f32) -> f32 {
        let t = if t <= 0.0 { 0.0 } else { t };
        loop {
            let s = &self.setpoint;
            if s.start_time_sec > t {
                self.rewind(ctrl);
            } else if s.end_time_sec < t {
                self.build_next(ctrl);
            } else if !t.is_finite() {
                return 1.0;
            } else if s.duration_sec.abs() > 1.0e-6 {
                return (t - s.start_time_sec) / s.duration_sec;
            } else {
                return 0.5;
            }
        }
    }

    /// Yaw in degrees at `t` seconds.
    pub fn yaw_at(&mut self, ctrl: &YawControl, t: f32) -> f32 {
        let rel = self.seek(ctrl, t);
        self.setpoint.start_yaw_deg + self.setpoint.yaw_change_deg * rel
    }

    /// Yaw rate in deg/s at `t` seconds (infinite for zero-length setpoints).
    pub fn yaw_rate_at(&mut self, ctrl: &YawControl, t: f32) -> f32 {
        self.seek(ctrl, t);
        let s = &self.setpoint;
        if s.duration_sec != 0.0 {
            s.yaw_change_deg / s.duration_sec
        } else {
            f32::INFINITY
        }
    }
}

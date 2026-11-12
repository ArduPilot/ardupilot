// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush rth_plan/rth_plan.c (Copyright CollMot Robotics Ltd).
//! Collective return-to-home plan (block 4): parser, evaluation at a show
//! time and conversion of a plan entry into a trajectory.

use alloc::vec::Vec;

use super::binary::ShowFile;
use super::builder::TrajectoryBuilder;
use super::fmath::ceilf;
use super::trajectory::Trajectory;
use super::{Error, Result, Vec3};

pub const BLOCK_RTH_PLAN: u8 = 4;

pub const ENTRY_HAS_NECK: u8 = 1 << 0;
pub const ENTRY_HAS_TARGET_XY: u8 = 1 << 1;
pub const ENTRY_HAS_TARGET_Z: u8 = 1 << 2;
pub const ENTRY_HAS_LANDING_ALTITUDE: u8 = 1 << 3;
pub const ENTRY_HAS_PRE_DELAY: u8 = 1 << 4;
pub const ENTRY_HAS_POST_DELAY: u8 = 1 << 5;

const HEADER_SIZE: usize = 10;
/// Largest integer exactly representable in f32.
const MAX_DURATION: u32 = 16777216;

/// What to do when the plan is triggered at a given time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RthPlanEntry {
    pub time_sec: f32,
    pub duration_sec: f32,
    pub flags: u8,
    pub landing_target: (f32, f32),
    pub arrival_altitude: f32,
    pub pre_delay_sec: f32,
    pub post_delay_sec: f32,
    pub pre_neck: f32,
    pub pre_neck_duration_sec: f32,
    pub max_acceleration: f32,
    pub landing_velocity: f32,
    pub landing_altitude: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RthPlan {
    buf: Vec<u8>,
    scale: f32,
    num_points: usize,
    max_acceleration: f32,
    landing_velocity: f32,
    landing_altitude: f32,
}

fn u16_at(b: &[u8], o: &mut usize) -> Result<u16> {
    let v = b.get(*o..*o + 2).ok_or(Error::Corrupted)?;
    *o += 2;
    Ok(u16::from_le_bytes([v[0], v[1]]))
}

/// sb_parse_varuint32: Parse at the end of the buffer, Overflow past 32 bits.
fn varuint32(b: &[u8], o: &mut usize) -> Result<u32> {
    let mut value: u32 = 0;
    let mut num_bits = 0u32;
    let mut bits_left = 32u32;
    let mut byte;
    loop {
        byte = *b.get(*o).ok_or(Error::Parse)?;
        *o += 1;
        if bits_left < 7 && (byte >> bits_left) > 0 {
            break;
        }
        value = value.wrapping_add(((byte & 0x7f) as u32) << num_bits);
        if byte & 0x80 == 0 {
            return Ok(value);
        }
        num_bits += 7;
        bits_left -= 7;
        if num_bits > 31 {
            break;
        }
    }
    loop {
        if byte & 0x80 == 0 {
            return Err(Error::Overflow);
        }
        byte = *b.get(*o).ok_or(Error::Parse)?;
        *o += 1;
    }
}

fn validate(b: &[u8]) -> Result<()> {
    if b.len() < HEADER_SIZE {
        return Err(Error::Corrupted);
    }
    let mut o = HEADER_SIZE - 2;
    let num_points = u16_at(b, &mut o)? as usize;
    if b.len() - o < num_points * 4 {
        return Err(Error::Corrupted);
    }
    o += num_points * 4;
    if o == b.len() {
        return Ok(());
    }
    let num_entries = u16_at(b, &mut o)?;
    let skip_i16 = |o: &mut usize| -> Result<()> {
        if b.len() - *o < 2 {
            return Err(Error::Corrupted);
        }
        *o += 2;
        Ok(())
    };
    // values that overflow 32 bits are reported on evaluation, not here
    let skip_varuint = |o: &mut usize| -> Result<()> {
        match varuint32(b, o) {
            Err(Error::Parse) => Err(Error::Corrupted),
            _ => Ok(()),
        }
    };
    for _ in 0..num_entries {
        let flags = *b.get(o).ok_or(Error::Corrupted)?;
        o += 1;
        skip_varuint(&mut o)?;
        if flags & ENTRY_HAS_NECK != 0 {
            skip_i16(&mut o)?;
            skip_varuint(&mut o)?;
        }
        if flags & ENTRY_HAS_TARGET_XY != 0 {
            skip_varuint(&mut o)?;
        }
        if flags & ENTRY_HAS_TARGET_Z != 0 {
            skip_i16(&mut o)?;
        }
        if flags & (ENTRY_HAS_TARGET_XY | ENTRY_HAS_TARGET_Z) != 0 {
            skip_varuint(&mut o)?;
        }
        if flags & ENTRY_HAS_LANDING_ALTITUDE != 0 {
            skip_i16(&mut o)?;
        }
        if flags & ENTRY_HAS_PRE_DELAY != 0 {
            skip_varuint(&mut o)?;
        }
        if flags & ENTRY_HAS_POST_DELAY != 0 {
            skip_varuint(&mut o)?;
        }
    }
    Ok(())
}

impl RthPlan {
    pub fn from_bytes(body: &[u8]) -> Result<Self> {
        validate(body)?;
        let scale = (body[1] & 0x7f) as f32;
        let mut o = 2;
        let max_acceleration = u16_at(body, &mut o)? as f32;
        let landing_velocity = u16_at(body, &mut o)? as f32;
        let landing_altitude = u16_at(body, &mut o)? as i16 as f32 * scale;
        let num_points = u16_at(body, &mut o)? as usize;
        Ok(RthPlan {
            buf: body.to_vec(),
            scale,
            num_points,
            max_acceleration: if max_acceleration == 0.0 { f32::INFINITY } else { max_acceleration },
            landing_velocity,
            landing_altitude,
        })
    }

    /// NoEnt if the file has no RTH plan block.
    pub fn from_show(file: &ShowFile) -> Result<Self> {
        Self::from_bytes(file.find_first(BLOCK_RTH_PLAN)?)
    }

    fn entry_table_offset(&self) -> usize {
        HEADER_SIZE + self.num_points * 4
    }

    pub fn num_entries(&self) -> usize {
        let mut o = self.entry_table_offset();
        u16_at(&self.buf, &mut o).map_or(0, |n| n as usize)
    }

    pub fn num_points(&self) -> usize {
        self.num_points
    }

    pub fn is_empty(&self) -> bool {
        self.num_entries() == 0
    }

    fn coordinate(&self, o: &mut usize) -> Result<f32> {
        Ok(u16_at(&self.buf, o)? as i16 as f32 * self.scale)
    }

    fn duration(&self, o: &mut usize) -> Result<f32> {
        let v = varuint32(&self.buf, o)?;
        if v > MAX_DURATION {
            return Err(Error::Overflow);
        }
        Ok(v as f32)
    }

    pub fn point(&self, index: usize) -> Result<(f32, f32)> {
        if index >= self.num_points {
            return Err(Error::Inval);
        }
        let mut o = HEADER_SIZE + index * 4;
        Ok((self.coordinate(&mut o)?, self.coordinate(&mut o)?))
    }

    pub fn default_acceleration_limit(&self) -> f32 {
        if self.max_acceleration > 0.0 && self.max_acceleration.is_finite() {
            self.max_acceleration
        } else {
            f32::INFINITY
        }
    }

    pub fn default_landing_velocity(&self) -> f32 {
        let v = self.landing_velocity;
        if v > 0.0 && v.is_finite() {
            v
        } else {
            f32::NAN
        }
    }

    pub fn default_landing_altitude(&self) -> f32 {
        self.landing_altitude
    }

    /// sb_rth_plan_entry_clear: land in place with the plan defaults.
    pub fn default_entry(plan: Option<&RthPlan>, time_sec: f32) -> RthPlanEntry {
        RthPlanEntry {
            time_sec,
            duration_sec: 0.0,
            flags: ENTRY_HAS_LANDING_ALTITUDE | ENTRY_HAS_PRE_DELAY | ENTRY_HAS_POST_DELAY | ENTRY_HAS_NECK,
            landing_target: (0.0, 0.0),
            arrival_altitude: 0.0,
            pre_delay_sec: 0.0,
            post_delay_sec: 0.0,
            pre_neck: 0.0,
            pre_neck_duration_sec: 0.0,
            max_acceleration: plan.map_or(f32::INFINITY, |p| p.default_acceleration_limit()),
            landing_velocity: plan.map_or(f32::NAN, |p| p.default_landing_velocity()),
            landing_altitude: plan.map_or(0.0, |p| p.default_landing_altitude()),
        }
    }

    /// Action for an RTH triggered at show time `time` (s): the first entry
    /// at or after `time`, or landing in place after the last entry.
    pub fn evaluate_at(&self, time: f32) -> Result<RthPlanEntry> {
        let num_entries = self.num_entries();
        let mut o = self.entry_table_offset() + 2;
        let mut time_s: u32 = 0;
        let mut entry = Self::default_entry(Some(self), time);
        let mut found = time < 0.0;
        let mut i = 0;
        while i < num_entries && !found {
            let flags = *self.buf.get(o).ok_or(Error::Corrupted)?;
            o += 1;
            let diff = varuint32(&self.buf, &mut o)?;
            time_s = time_s.checked_add(diff).ok_or(Error::Overflow)?;
            entry.time_sec = time_s as f32;
            if flags != 0 {
                entry = Self::default_entry(Some(self), time_s as f32);
                if flags & ENTRY_HAS_NECK != 0 {
                    entry.pre_neck = self.coordinate(&mut o)?;
                    entry.pre_neck_duration_sec = self.duration(&mut o)?;
                }
                if flags & ENTRY_HAS_TARGET_XY != 0 {
                    let idx = varuint32(&self.buf, &mut o)?;
                    entry.landing_target = self.point(idx as usize)?;
                }
                if flags & ENTRY_HAS_TARGET_Z != 0 {
                    entry.arrival_altitude = self.coordinate(&mut o)?;
                }
                if flags & (ENTRY_HAS_TARGET_XY | ENTRY_HAS_TARGET_Z) != 0 {
                    entry.duration_sec = self.duration(&mut o)?;
                }
                if flags & ENTRY_HAS_LANDING_ALTITUDE != 0 {
                    entry.landing_altitude = self.coordinate(&mut o)?;
                }
                if flags & ENTRY_HAS_PRE_DELAY != 0 {
                    entry.pre_delay_sec = self.duration(&mut o)?;
                }
                if flags & ENTRY_HAS_POST_DELAY != 0 {
                    entry.post_delay_sec = self.duration(&mut o)?;
                }
                entry.flags |= flags;
            }
            if time_s as f32 >= time {
                found = true;
            }
            i += 1;
        }
        if !found {
            entry = Self::default_entry(Some(self), time);
        }
        Ok(entry)
    }
}

/// sb_i_scale_update
fn scale_update(scale: &mut u8, x: f32, y: f32, z: f32) -> Result<()> {
    if *scale == 0 {
        *scale = 1;
    }
    let max = *scale as f32 * i16::MAX as f32;
    let mut m = x.abs();
    if y.abs() > m {
        m = y.abs();
    }
    if z.abs() > m {
        m = z.abs();
    }
    if m > max {
        let s = ceilf(m / i16::MAX as f32);
        if s <= 127.0 {
            *scale = s as u8;
        } else {
            return Err(Error::Overflow);
        }
    }
    Ok(())
}

/// sb_uint32_msec_duration_from_float_seconds
pub fn msec_from_sec(sec: f32) -> Result<u32> {
    if sec < 0.0 || sec.is_nan() {
        return Err(Error::Inval);
    }
    let ms = sec * 1000.0;
    if !(ms < 4294967296.0) {
        return Err(Error::Overflow);
    }
    Ok(ms as u32)
}

/// sb_trajectory_update_from_rth_plan_entry: hold until the entry time plus
/// pre-delay, neck, move to the target, post-delay, then descend to the
/// landing altitude if a landing velocity is given.
pub fn trajectory_from_entry(entry: &RthPlanEntry, start: Vec3) -> Result<Trajectory> {
    let mut max_accel = entry.max_acceleration;
    if max_accel <= 0.0 || !max_accel.is_finite() {
        max_accel = f32::INFINITY;
    }
    let mut scale = 1u8;
    scale_update(&mut scale, start.x, start.y, start.z)?;
    if entry.flags & ENTRY_HAS_NECK != 0 {
        scale_update(&mut scale, 0.0, 0.0, start.z + entry.pre_neck)?;
    }
    if entry.flags & ENTRY_HAS_TARGET_XY != 0 {
        scale_update(&mut scale, entry.landing_target.0, entry.landing_target.1, 0.0)?;
    }
    if entry.flags & ENTRY_HAS_TARGET_Z != 0 {
        scale_update(&mut scale, 0.0, 0.0, entry.arrival_altitude)?;
    }
    scale_update(&mut scale, 0.0, 0.0, entry.landing_altitude)?;

    let mut start_time = entry.time_sec;
    if start_time < 0.0 {
        start_time = 0.0;
    }
    let mut b = TrajectoryBuilder::new(scale)?;
    b.set_start_position(start)?;
    let pre = if entry.pre_delay_sec > 0.0 { entry.pre_delay_sec } else { 0.0 };
    b.hold_position_for(msec_from_sec(start_time + pre)?)?;

    let mut target = start;
    if entry.pre_neck != 0.0 || entry.pre_neck_duration_sec != 0.0 {
        let d = msec_from_sec(entry.pre_neck_duration_sec)?;
        target.z += entry.pre_neck;
        b.move_to_in_time(target, d.max(1), max_accel)?;
    }
    if entry.flags & ENTRY_HAS_TARGET_XY != 0 {
        target.x = entry.landing_target.0;
        target.y = entry.landing_target.1;
    }
    if entry.flags & ENTRY_HAS_TARGET_Z != 0 {
        target.z = entry.arrival_altitude;
    }
    if entry.flags & (ENTRY_HAS_TARGET_XY | ENTRY_HAS_TARGET_Z) != 0 {
        let d = msec_from_sec(entry.duration_sec)?;
        b.move_to_in_time(target, d.max(1), max_accel)?;
    }
    b.hold_position_for(msec_from_sec(entry.post_delay_sec)?)?;

    if entry.landing_velocity > 0.0 && entry.landing_velocity.is_finite() {
        let current = target.z;
        target.z = entry.landing_altitude;
        let d = msec_from_sec((current - target.z).abs() / entry.landing_velocity)?;
        b.move_to_in_time(target, d.max(1), max_accel)?;
    }
    b.into_trajectory()
}

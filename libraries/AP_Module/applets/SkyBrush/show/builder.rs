// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush trajectory/builder.c, motion.c and the landing
// replacement in trajectory.c (Copyright CollMot Robotics Ltd).
//! Trajectory builder, motion helpers and landing replacement.

use alloc::vec::Vec;

use super::fmath::{hypotf, sqrtf};
use super::stats::{Stats, STATS_DURATION, STATS_LANDING_TIME, STATS_START_END_DISTANCE};
use super::trajectory::{scale_coordinate, Trajectory, HEADER_LENGTH};
use super::{Error, Result, Vec3};

const MAX_DURATION_MSEC: u32 = 60000;
const X_LINEAR: u8 = 0x01;
const X_BEZIER: u8 = 0x02;

/// Control points of the cubic Bezier with the given end velocities.
pub fn cubic_bezier_from_velocity_constraints(
    start: Vec3,
    start_vel: Vec3,
    end: Vec3,
    end_vel: Vec3,
    duration_sec: f32,
) -> Result<(Vec3, Vec3)> {
    if duration_sec <= 0.0 || duration_sec.is_nan() {
        return Err(Error::Inval);
    }
    let s = duration_sec / 3.0;
    let c1 = Vec3::new(start.x + s * start_vel.x, start.y + s * start_vel.y, start.z + s * start_vel.z);
    let c2 = Vec3::new(end.x - s * end_vel.x, end.y - s * end_vel.y, end.z - s * end_vel.z);
    Ok((c1, c2))
}

/// Time to travel `distance` from and to standstill with the given limits.
pub fn travel_time_for_distance(distance: f32, max_speed: f32, acceleration: f32) -> f32 {
    if distance < 0.0 || max_speed <= 0.0 || acceleration <= 0.0 {
        return f32::INFINITY;
    }
    if distance == 0.0 {
        return 0.0;
    }
    if acceleration == f32::INFINITY {
        return distance / max_speed;
    }
    let mut t1 = max_speed / acceleration;
    let s1 = acceleration / 2.0 * t1 * t1;
    let t2;
    if distance >= 2.0 * s1 {
        t2 = (distance - 2.0 * s1) / max_speed;
    } else {
        t1 = sqrtf(distance / acceleration);
        t2 = 0.0;
    }
    2.0 * t1 + t2
}

/// Cruise velocity needed to travel `distance` in `time`.
pub fn travel_velocity_for_distance(distance: f32, time: f32, acceleration: f32) -> f32 {
    if distance < 0.0 || time <= 0.0 || acceleration <= 0.0 {
        return f32::INFINITY;
    }
    if distance == 0.0 {
        return 0.0;
    }
    if acceleration == f32::INFINITY {
        return distance / time;
    }
    let d = time * time - 4.0 * distance / acceleration;
    if d >= 0.0 {
        -(sqrtf(d) - time) / 2.0 * acceleration
    } else {
        f32::INFINITY
    }
}

/// Appends segments to a trajectory buffer. Every append is atomic: on error
/// the builder is left as it was.
#[derive(Clone, Debug)]
pub struct TrajectoryBuilder {
    buf: Vec<u8>,
    last_position: Vec3,
    scale: f32,
}

impl TrajectoryBuilder {
    pub fn new(scale: u8) -> Result<Self> {
        if scale == 0 || scale > 127 {
            return Err(Error::Inval);
        }
        let mut buf = alloc::vec![0u8; HEADER_LENGTH];
        buf[0] = scale;
        Ok(TrajectoryBuilder { buf, last_position: Vec3::ZERO, scale: scale as f32 })
    }

    /// Continues an existing trajectory; `last_position` defaults to its end.
    pub fn from_trajectory(traj: Trajectory, last_position: Option<Vec3>) -> Self {
        let last_position = last_position.unwrap_or_else(|| traj.end_position());
        TrajectoryBuilder { scale: traj.scale, buf: traj.buf, last_position }
    }

    pub fn last_position(&self) -> Vec3 {
        self.last_position
    }

    pub fn set_start_position(&mut self, start: Vec3) -> Result<()> {
        if self.buf.len() != HEADER_LENGTH {
            return Err(Error::Failure);
        }
        let x = scale_coordinate(self.scale, start.x)?;
        let y = scale_coordinate(self.scale, start.y)?;
        let z = scale_coordinate(self.scale, start.z)?;
        self.buf.truncate(1);
        for v in [x, y, z, 0] {
            self.buf.extend_from_slice(&v.to_le_bytes());
        }
        self.last_position = start;
        Ok(())
    }

    fn atomic(&mut self, f: impl FnOnce(&mut Self) -> Result<()>) -> Result<()> {
        let (len, last) = (self.buf.len(), self.last_position);
        let r = f(self);
        if r.is_err() {
            self.buf.truncate(len);
            self.last_position = last;
        }
        r
    }

    fn push_coord(&mut self, v: f32) -> Result<()> {
        let s = scale_coordinate(self.scale, v)?;
        self.buf.extend_from_slice(&s.to_le_bytes());
        Ok(())
    }

    fn push_header(&mut self, duration_msec: u32) -> usize {
        let at = self.buf.len();
        self.buf.push(0);
        self.buf.extend_from_slice(&(duration_msec as u16).to_le_bytes());
        at
    }

    pub fn append_cubic_bezier(&mut self, c1: Vec3, c2: Vec3, target: Vec3, duration_msec: u32) -> Result<()> {
        self.atomic(|b| b.cubic_bezier(c1, c2, target, duration_msec))
    }

    fn cubic_bezier(&mut self, c1: Vec3, c2: Vec3, target: Vec3, duration_msec: u32) -> Result<()> {
        if duration_msec == 0 || duration_msec > MAX_DURATION_MSEC {
            return Err(Error::Inval);
        }
        let at = self.push_header(duration_msec);
        let p = self.last_position;
        let axes = [(p.x, c1.x, c2.x, target.x), (p.y, c1.y, c2.y, target.y), (p.z, c1.z, c2.z, target.z)];
        for (axis, &(p, a, b, t)) in axes.iter().enumerate() {
            if p != a || a != b || b != t {
                self.buf[at] |= X_BEZIER << (2 * axis);
                self.push_coord(a)?;
                self.push_coord(b)?;
                self.push_coord(t)?;
            }
        }
        self.last_position = target;
        Ok(())
    }

    pub fn append_line(&mut self, target: Vec3, duration_msec: u32) -> Result<()> {
        self.atomic(|b| b.line(target, duration_msec))
    }

    fn line(&mut self, target: Vec3, duration_msec: u32) -> Result<()> {
        if duration_msec == 0 {
            return Err(Error::Inval);
        }
        if duration_msec > MAX_DURATION_MSEC {
            let p = self.last_position;
            let mid = Vec3::new((p.x + target.x) / 2.0, (p.y + target.y) / 2.0, (p.z + target.z) / 2.0);
            let half = duration_msec >> 1;
            self.line(mid, half)?;
            return self.line(target, duration_msec - half);
        }
        let at = self.push_header(duration_msec);
        let p = self.last_position;
        for (axis, (p, t)) in [(p.x, target.x), (p.y, target.y), (p.z, target.z)].into_iter().enumerate() {
            if p != t {
                self.buf[at] |= X_LINEAR << (2 * axis);
                self.push_coord(t)?;
            }
        }
        self.last_position = target;
        Ok(())
    }

    pub fn hold_position_for(&mut self, duration_msec: u32) -> Result<()> {
        self.atomic(|b| b.hold(duration_msec))
    }

    fn hold(&mut self, mut duration_msec: u32) -> Result<()> {
        while duration_msec > 0 {
            let d = duration_msec.min(MAX_DURATION_MSEC);
            duration_msec -= d;
            self.line(self.last_position, d)?;
        }
        Ok(())
    }

    /// Trapezoidal-velocity move: cubic acceleration, cruise line, cubic
    /// deceleration. Non-positive or infinite `max_acceleration` moves in a line.
    pub fn move_to_in_time(&mut self, target: Vec3, duration_msec: u32, max_acceleration: f32) -> Result<()> {
        self.atomic(|b| b.move_to(target, duration_msec, max_acceleration))
    }

    fn move_to(&mut self, target: Vec3, duration_msec: u32, max_accel: f32) -> Result<()> {
        if duration_msec == 0 {
            return Err(Error::Inval);
        }
        let p = self.last_position;
        let distance = hypotf(hypotf(target.x - p.x, target.y - p.y), target.z - p.z);
        if distance == 0.0 {
            return self.hold(duration_msec);
        }
        if !max_accel.is_finite() || max_accel <= 0.0 || duration_msec < 3 {
            return self.line(target, duration_msec);
        }
        let duration_sec = duration_msec as f32 / 1000.0;
        let mut v2 = travel_velocity_for_distance(distance, duration_sec, max_accel);
        let mut t2 = duration_sec - 2.0 * v2 / max_accel;
        if t2 < 0.0 {
            t2 = 0.0;
            v2 = 2.0 * distance / duration_sec;
        }
        let mut t1 = (duration_sec - t2) / 2.0;
        let mut t1_msec = (t1 * 1000.0) as u32;
        if t1_msec < 1 {
            t1_msec = 1;
            t1 = t1_msec as f32 / 1000.0;
        }
        let vel = Vec3::new(
            v2 * (target.x - p.x) / distance,
            v2 * (target.y - p.y) / distance,
            v2 * (target.z - p.z) / distance,
        );
        let q = Vec3::new(p.x + vel.x * t1 / 2.0, p.y + vel.y * t1 / 2.0, p.z + vel.z * t1 / 2.0);
        let (c1, c2) = cubic_bezier_from_velocity_constraints(p, Vec3::ZERO, q, vel, t1)?;
        self.cubic_bezier(c1, c2, q, t1_msec)?;

        let p = Vec3::new(target.x - vel.x * t1 / 2.0, target.y - vel.y * t1 / 2.0, target.z - vel.z * t1 / 2.0);
        if duration_msec > 2 * t1_msec {
            self.line(p, duration_msec - 2 * t1_msec)?;
        }
        let (c1, c2) = cubic_bezier_from_velocity_constraints(p, vel, target, Vec3::ZERO, t1)?;
        self.cubic_bezier(c1, c2, target, t1_msec)
    }

    pub fn bytes(&self) -> &[u8] {
        &self.buf
    }

    pub fn into_trajectory(self) -> Result<Trajectory> {
        Trajectory::from_bytes(&self.buf)
    }
}

impl Trajectory {
    /// Replaces the end of the trajectory after `stats.landing_time_sec` with
    /// one cubic segment landing at `pos` (descending at `landing_velocity`,
    /// arriving with `terminal_velocity` downwards) and updates `stats`.
    /// The trajectory and stats are unchanged on error.
    pub fn replace_end_to_land_at(
        &mut self,
        stats: &mut Stats,
        pos: Vec3,
        landing_velocity: f32,
        terminal_velocity: f32,
    ) -> Result<()> {
        if stats.valid_components & STATS_LANDING_TIME == 0 {
            return Err(Error::Inval);
        }
        if terminal_velocity < 0.0 || landing_velocity <= 0.0 {
            return Err(Error::Inval);
        }
        let vertical = stats.pos_at_landing_time.z - pos.z;
        let mut duration_sec = if vertical < 0.0 { 0.0 } else { vertical / landing_velocity };
        if duration_sec > 60.0 {
            duration_sec = 60.0;
        }
        let duration_msec = (duration_sec * 1000.0) as u32;
        if duration_msec == 0 {
            return Err(Error::Inval);
        }
        let end_vel = Vec3::new(0.0, 0.0, -terminal_velocity);
        let (c1, c2) = cubic_bezier_from_velocity_constraints(
            stats.pos_at_landing_time,
            stats.vel_at_landing_time,
            pos,
            end_vel,
            duration_sec,
        )?;

        // build the segment first so that a failure leaves everything intact
        let mut b = TrajectoryBuilder::new(self.scale as u8)?;
        b.last_position = stats.pos_at_landing_time;
        b.append_cubic_bezier(c1, c2, pos, duration_msec)?;

        self.cut_at(stats.landing_time_sec)?;
        if stats.valid_components & STATS_DURATION != 0 {
            stats.duration_sec = stats.landing_time_sec;
            stats.duration_msec = (stats.duration_sec * 1000.0) as u32;
        }
        self.buf.extend_from_slice(&b.buf[HEADER_LENGTH..]);

        stats.landing_time_sec += duration_sec;
        stats.pos_at_landing_time = pos;
        stats.vel_at_landing_time = end_vel;
        if stats.valid_components & STATS_DURATION != 0 {
            stats.duration_sec += duration_sec;
            stats.duration_msec = stats.duration_msec.saturating_add(duration_msec);
        }
        if stats.valid_components & STATS_START_END_DISTANCE != 0 {
            stats.start_to_end_distance_xy = hypotf(pos.x - self.start.x, pos.y - self.start.y);
        }
        Ok(())
    }
}

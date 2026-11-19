// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush trajectory/stats.c (Copyright CollMot Robotics Ltd).
//! Trajectory statistics: duration, takeoff and landing time, start-to-end
//! XY distance.

use super::builder::travel_time_for_distance;
use super::fmath::hypotf;
use super::trajectory::{Player, Segment, Trajectory};
use super::{Error, Result, Vec3};

pub const STATS_DURATION: u8 = 1;
pub const STATS_START_END_DISTANCE: u8 = 2;
pub const STATS_TAKEOFF_TIME: u8 = 4;
pub const STATS_LANDING_TIME: u8 = 8;
pub const STATS_ALL: u8 = 15;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stats {
    pub duration_msec: u32,
    pub duration_sec: f32,
    /// Earliest time the trajectory reaches start.z + min_ascent.
    pub earliest_above_sec: f32,
    /// Infinity when never above the takeoff altitude.
    pub takeoff_time_sec: f32,
    pub landing_time_sec: f32,
    pub initial_pos: Vec3,
    pub pos_at_landing_time: Vec3,
    pub vel_at_landing_time: Vec3,
    pub start_to_end_distance_xy: f32,
    pub valid_components: u8,
}

impl Default for Stats {
    fn default() -> Self {
        Stats {
            duration_msec: 0,
            duration_sec: 0.0,
            earliest_above_sec: f32::INFINITY,
            takeoff_time_sec: f32::INFINITY,
            landing_time_sec: 0.0,
            initial_pos: Vec3::ZERO,
            pos_at_landing_time: Vec3::ZERO,
            vel_at_landing_time: Vec3::ZERO,
            start_to_end_distance_xy: 0.0,
            valid_components: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StatsCalculator {
    pub components: u8,
    /// Assumed takeoff speed, units/s.
    pub takeoff_speed: f32,
    /// Assumed vertical takeoff acceleration, units/s^2.
    pub acceleration: f32,
    pub min_ascent: f32,
    /// Length of the final autonomous descent.
    pub preferred_descent: f32,
    /// XY tolerance for a segment to count as vertical.
    pub verticality_threshold: f32,
}

impl StatsCalculator {
    /// Defaults of sb_trajectory_stats_calculator_init; `scale` is units per metre.
    pub fn new(scale: f32) -> Self {
        StatsCalculator {
            components: STATS_ALL,
            acceleration: 4.0 * scale,
            takeoff_speed: 2.0 * scale,
            min_ascent: 2.5 * scale,
            preferred_descent: 2.5 * scale,
            verticality_threshold: scale * 0.05,
        }
    }

    /// Configuration used by the Skybrush ArduPilot fork (mm units).
    pub fn for_show(takeoff_alt_mm: f32, takeoff_speed_mm_s: f32, takeoff_accel_mm_s2: f32) -> Self {
        StatsCalculator {
            min_ascent: takeoff_alt_mm,
            preferred_descent: takeoff_alt_mm,
            takeoff_speed: takeoff_speed_mm_s,
            acceleration: takeoff_accel_mm_s2,
            ..Self::new(1000.0)
        }
    }

    pub fn run(&self, traj: &Trajectory) -> Result<Stats> {
        if self.acceleration <= 0.0
            || !self.takeoff_speed.is_finite()
            || self.takeoff_speed <= 0.0
            || !self.min_ascent.is_finite()
            || self.min_ascent < 0.0
            || !self.preferred_descent.is_finite()
            || self.preferred_descent < 0.0
        {
            return Err(Error::Inval);
        }
        let mut res = Stats::default();
        let mut components = self.components;
        if components == 0 {
            return Ok(res);
        }

        let mut player = Player::new(traj);
        let start = player.position_at(traj, 0.0);
        res.initial_pos = start;
        let takeoff_altitude = start.z + self.min_ascent;

        let mut saved = None;
        let mut vstart_alt = 0.0f32;
        let mut vend_alt = 0.0f32;
        let mut landing_recorded = false;

        while player.has_more_segments() {
            let seg = player.current_segment();
            if components & STATS_DURATION != 0 {
                res.duration_msec = res.duration_msec.saturating_add(seg.duration_msec);
            }
            if components & STATS_TAKEOFF_TIME != 0 {
                if let Some(rel) = seg.poly(traj).z.touches(takeoff_altitude) {
                    res.earliest_above_sec = seg.start_time_sec + rel * seg.duration_sec;
                    components &= !STATS_TAKEOFF_TIME;
                    if components == 0 {
                        break;
                    }
                }
            }
            if components & STATS_LANDING_TIME != 0 {
                if is_descending_vertically(seg, self.verticality_threshold) {
                    if saved.is_none() {
                        vstart_alt = seg.start.z;
                        saved = Some(player.state);
                    }
                    vend_alt = player.state.segment.end.z;
                } else {
                    record_landing(&mut res, seg, traj, 1.0, true);
                    landing_recorded = true;
                    saved = None;
                }
            }
            player.build_next_segment(traj);
        }

        let components = self.components;
        if components & STATS_DURATION != 0 {
            res.duration_sec = res.duration_msec as f32 / 1000.0;
            res.valid_components |= STATS_DURATION;
        }
        if components & STATS_TAKEOFF_TIME != 0 {
            let adj = travel_time_for_distance(self.min_ascent, self.takeoff_speed, self.acceleration);
            res.takeoff_time_sec = if res.earliest_above_sec.is_finite() && adj.is_finite() {
                res.earliest_above_sec - adj
            } else {
                f32::INFINITY
            };
            res.valid_components |= STATS_TAKEOFF_TIME;
        }
        if components & STATS_START_END_DISTANCE != 0 {
            let end = player.position_at(traj, f32::INFINITY);
            res.start_to_end_distance_xy = hypotf(end.x - start.x, end.y - start.y);
            res.valid_components |= STATS_START_END_DISTANCE;
        }
        if components & STATS_LANDING_TIME != 0 {
            let (mut to_descend, mut landing_found) = match saved {
                Some(state) => {
                    player.state = state;
                    (vstart_alt - (vend_alt + self.preferred_descent), false)
                }
                None => (0.0, landing_recorded),
            };
            if to_descend > 0.0 {
                let mut altitude = player.state.segment.start.z;
                while player.has_more_segments() {
                    let seg = player.current_segment();
                    let delta = altitude - seg.end.z;
                    if delta < 0.0 {
                        // ascending inside a vertical section; cannot happen
                        return Ok(res);
                    } else if delta <= to_descend {
                        to_descend -= delta;
                        altitude = seg.end.z;
                        record_landing(&mut res, seg, traj, 1.0, true);
                        landing_found = true;
                    } else {
                        let rel = seg.poly(traj).z.touches(altitude - to_descend).unwrap_or(0.0);
                        record_landing(&mut res, seg, traj, rel, false);
                        landing_found = true;
                        break;
                    }
                    player.build_next_segment(traj);
                }
            }
            if !landing_found {
                let seg = player.current_segment();
                res.landing_time_sec = seg.start_time_sec;
                res.pos_at_landing_time = seg.start;
                res.vel_at_landing_time = seg.dpoly(traj).eval(0.0);
            }
            res.valid_components |= STATS_LANDING_TIME;
        }
        Ok(res)
    }
}

fn record_landing(res: &mut Stats, seg: &mut Segment, traj: &Trajectory, rel: f32, at_end: bool) {
    if at_end {
        res.landing_time_sec = seg.end_time_sec;
        res.pos_at_landing_time = seg.end;
    } else {
        res.landing_time_sec = seg.start_time_sec + rel * seg.duration_sec;
        res.pos_at_landing_time = seg.poly(traj).eval(rel);
    }
    res.vel_at_landing_time = seg.dpoly(traj).eval(rel);
}

fn is_descending_vertically(seg: &Segment, threshold: f32) -> bool {
    if seg.start.z < seg.end.z {
        return false;
    }
    let dx = (seg.start.x - seg.end.x).abs();
    let dy = (seg.start.y - seg.end.y).abs();
    let dz = seg.start.z - seg.end.z;
    dx <= threshold && dy <= threshold && dz >= dx && dz >= dy
}

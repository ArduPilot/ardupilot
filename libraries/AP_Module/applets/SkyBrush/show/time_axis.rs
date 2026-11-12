// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush screenplay/time_axis.c (Copyright CollMot Robotics Ltd).
//! Piecewise time warping: wall clock ms -> warped (show) seconds. Each
//! segment changes the rate linearly; before the origin, without segments,
//! and after the last segment time flows at the boundary rate.

use alloc::vec::Vec;

use super::fmath::{ceilf, sqrtf};
use super::{Error, Result};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimeSegment {
    /// Wall clock duration; u32::MAX is infinite.
    pub duration_msec: u32,
    pub initial_rate: f32,
    pub final_rate: f32,
}

impl TimeSegment {
    pub fn new(duration_msec: u32, initial_rate: f32, final_rate: f32) -> Self {
        TimeSegment { duration_msec, initial_rate, final_rate }
    }

    pub fn realtime(duration_msec: u32) -> Self {
        Self::new(duration_msec, 1.0, 1.0)
    }

    /// Segment covering `warped_duration_sec` of show time; `None` if invalid.
    pub fn warped(warped_duration_sec: f32, initial_rate: f32, final_rate: f32) -> Option<Self> {
        if warped_duration_sec < 0.0
            || initial_rate < 0.0
            || final_rate < 0.0
            || warped_duration_sec.is_nan()
            || !initial_rate.is_finite()
            || !final_rate.is_finite()
        {
            return None;
        }
        if warped_duration_sec == 0.0 {
            return Some(Self::new(0, initial_rate, final_rate));
        }
        let avg = (initial_rate + final_rate) / 2.0;
        if avg == 0.0 {
            return Some(Self::new(u32::MAX, initial_rate, final_rate));
        }
        Some(Self::new(ceilf(warped_duration_sec / avg * 1000.0) as u32, initial_rate, final_rate))
    }

    pub fn wall_duration_sec(&self) -> f32 {
        if self.duration_msec == u32::MAX {
            f32::INFINITY
        } else {
            self.duration_msec as f32 / 1000.0
        }
    }

    pub fn warped_duration_sec(&self) -> f32 {
        let wall = self.wall_duration_sec();
        if wall.is_infinite() {
            f32::INFINITY
        } else {
            wall * ((self.initial_rate + self.final_rate) / 2.0)
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct TimeAxis {
    origin_msec: i32,
    segments: Vec<TimeSegment>,
}

impl TimeAxis {
    pub fn origin_msec(&self) -> i32 {
        self.origin_msec
    }

    pub fn set_origin_msec(&mut self, origin_msec: i32) -> Result<()> {
        if origin_msec < 0 {
            return Err(Error::Inval);
        }
        self.origin_msec = origin_msec;
        Ok(())
    }

    pub fn segments(&self) -> &[TimeSegment] {
        &self.segments
    }

    pub fn clear(&mut self) {
        self.segments.clear();
    }

    pub fn push(&mut self, seg: TimeSegment) -> Result<()> {
        if seg.initial_rate.is_nan() || seg.final_rate.is_nan() || seg.initial_rate < 0.0 || seg.final_rate < 0.0 {
            return Err(Error::Inval);
        }
        self.segments.push(seg);
        Ok(())
    }

    /// Total wall clock duration, u32::MAX if infinite or overflowing.
    pub fn total_duration_msec(&self) -> u32 {
        let mut total = 0u32;
        for s in &self.segments {
            if s.duration_msec == u32::MAX {
                return u32::MAX;
            }
            total = match total.checked_add(s.duration_msec) {
                Some(t) => t,
                None => return u32::MAX,
            };
        }
        total
    }

    pub fn total_warped_duration_sec(&self) -> f32 {
        self.segments.iter().fold(0.0, |acc, s| acc + s.warped_duration_sec())
    }

    /// Warped time (s) and rate at wall clock time `wall_msec`.
    pub fn map(&self, wall_msec: i32) -> (f32, f32) {
        if wall_msec < self.origin_msec || self.segments.is_empty() {
            return (wall_msec.wrapping_sub(self.origin_msec) as f32 / 1000.0, 1.0);
        }
        let mut t = (wall_msec - self.origin_msec) as u32;
        let mut acc = 0.0f32;
        for seg in &self.segments {
            let d = seg.duration_msec;
            let infinite = d == u32::MAX;
            if infinite || d > t {
                let (w, rate) = if infinite || seg.initial_rate == seg.final_rate {
                    (t as f32 / 1000.0 * seg.initial_rate, seg.initial_rate)
                } else if d > 0 {
                    let delta = seg.final_rate - seg.initial_rate;
                    let rel = t as f32 / d as f32;
                    ((seg.initial_rate + delta / 2.0 * rel) * t as f32 / 1000.0, seg.initial_rate + delta * rel)
                } else {
                    (0.0, (seg.initial_rate + seg.final_rate) / 2.0)
                };
                return (acc + w, rate);
            }
            t -= d;
            acc += seg.warped_duration_sec();
        }
        let last = self.segments[self.segments.len() - 1];
        (acc + t as f32 / 1000.0 * last.final_rate, last.final_rate)
    }

    /// Wall clock ms at which warped time `time` (s) is reached.
    pub fn reverse_map(&self, time: f32) -> i32 {
        let mut acc = 0.0f32;
        if !time.is_nan() {
            if time.is_infinite() {
                return if time < 0.0 { i32::MIN } else { i32::MAX };
            }
            let mut rem = time;
            if rem < 0.0 || self.segments.is_empty() {
                acc = rem;
            } else {
                for seg in &self.segments {
                    if rem <= 0.0 {
                        break;
                    }
                    let wd = seg.warped_duration_sec();
                    if rem < wd {
                        if !wd.is_finite() || seg.initial_rate == seg.final_rate {
                            acc += if seg.initial_rate == 0.0 {
                                if rem > 0.0 { f32::INFINITY } else { 0.0 }
                            } else {
                                rem / seg.initial_rate
                            };
                        } else if wd > 0.0 {
                            let a = (seg.final_rate - seg.initial_rate) / (2.0 * seg.wall_duration_sec());
                            let mut roots = [0.0f32; 2];
                            match solve_quadratic(a, seg.initial_rate, -rem, &mut roots) {
                                0 => {}
                                1 => acc += roots[0],
                                _ => acc += if roots[0] >= 0.0 { roots[0] } else { roots[1] },
                            }
                        }
                        rem = 0.0;
                    } else {
                        rem -= wd;
                        acc += seg.wall_duration_sec();
                    }
                }
                if rem > 0.0 {
                    let last = self.segments[self.segments.len() - 1];
                    acc += if last.final_rate == 0.0 { f32::INFINITY } else { rem / last.final_rate };
                }
            }
        }
        if !acc.is_finite() {
            return if acc < 0.0 { i32::MIN } else { i32::MAX };
        }
        // float -> int32 saturates here; C would be undefined on overflow
        self.origin_msec.saturating_add((acc * 1000.0) as i32)
    }
}

/// Real roots of a*t^2 + b*t + c = 0, smaller first when a > 0.
pub fn solve_quadratic(a: f32, b: f32, c: f32, roots: &mut [f32; 2]) -> usize {
    if a.abs() < f32::MIN_POSITIVE {
        if b.abs() < f32::MIN_POSITIVE {
            return 0;
        }
        roots[0] = -c / b;
        return 1;
    }
    let d = b * b - 4.0 * a * c;
    if d.abs() < f32::MIN_POSITIVE {
        roots[0] = -b / (2.0 * a);
        return 1;
    }
    if d > 0.0 {
        let d = sqrtf(d);
        roots[0] = (-b - d) / (2.0 * a);
        roots[1] = (-b + d) / (2.0 * a);
        if a < 0.0 {
            roots.swap(0, 1);
        }
        return 2;
    }
    0
}

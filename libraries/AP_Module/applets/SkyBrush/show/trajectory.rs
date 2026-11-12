// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush trajectory/{trajectory,segment,player}.c (Copyright CollMot Robotics Ltd).
//! Trajectory block (type 1) and its player.
//!
//! Header: scale u8 (bit7 ignored, 0 = empty), start x/y/z i16, legacy yaw i16.
//! Segment: format u8 (2 bits per axis X,Y,Z,yaw: 1/2/4/8 points), duration
//! u16 ms, then the control points of X, Y, Z, yaw (i16 * scale, mm) without
//! the implicit start point.

use alloc::vec::Vec;

use super::binary::{read_i16, read_u16, write_u16, ShowFile, BLOCK_TRAJECTORY};
use super::fmath::roundf;
use super::poly::{bezier_cut_at, Poly, Poly3};
use super::{Error, Result, Vec3};

pub(crate) const HEADER_LENGTH: usize = 9;

pub(crate) fn num_coords(header_bits: u8) -> usize {
    1 << (header_bits & 3)
}

pub(crate) fn scale_coordinate(scale: f32, coordinate: f32) -> Result<i16> {
    let scaled = super::fmath::floorf(coordinate / scale);
    if scaled >= i16::MIN as f32 && scaled <= i16::MAX as f32 {
        Ok(scaled as i16)
    } else {
        Err(Error::Inval)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Trajectory {
    pub(crate) buf: Vec<u8>,
    pub(crate) start: Vec3,
    pub(crate) scale: f32,
    pub(crate) header_length: usize,
}

impl Default for Trajectory {
    fn default() -> Self {
        Trajectory { buf: Vec::new(), start: Vec3::ZERO, scale: 1.0, header_length: 0 }
    }
}

impl Trajectory {
    /// Copies and validates the body of a trajectory block.
    pub fn from_bytes(body: &[u8]) -> Result<Self> {
        validate(body)?;
        let mut t = Trajectory { buf: body.to_vec(), ..Default::default() };
        t.parse_header();
        Ok(t)
    }

    /// Loads the first trajectory block of a show file.
    pub fn from_show(file: &ShowFile) -> Result<Self> {
        Self::from_bytes(file.find_first(BLOCK_TRAJECTORY)?)
    }

    fn parse_header(&mut self) {
        self.scale = (self.buf[0] & 0x7f) as f32;
        let mut off = 1;
        self.start.x = self.coord(&mut off);
        self.start.y = self.coord(&mut off);
        self.start.z = self.coord(&mut off);
        self.header_length = off + 2;
    }

    pub(crate) fn coord(&self, off: &mut usize) -> f32 {
        read_i16(&self.buf, off) as f32 * self.scale
    }

    pub fn bytes(&self) -> &[u8] {
        &self.buf
    }

    pub fn start(&self) -> Vec3 {
        self.start
    }

    pub fn scale(&self) -> f32 {
        self.scale
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty() || self.buf[0] & 0x7f == 0
    }

    pub fn total_duration_msec(&self) -> u32 {
        let mut p = Player::new(self);
        let mut total = 0u32;
        while p.has_more_segments() {
            total = total.saturating_add(p.state.segment.duration_msec);
            p.build_next_segment(self);
        }
        total
    }

    pub fn total_duration_sec(&self) -> f32 {
        self.total_duration_msec() as f32 / 1000.0
    }

    pub fn start_position(&self) -> Vec3 {
        Player::new(self).position_at(self, 0.0)
    }

    pub fn end_position(&self) -> Vec3 {
        Player::new(self).position_at(self, f32::INFINITY)
    }

    /// Player state with the segment that contains `t`, and the relative time.
    pub fn segment_at(&self, t: f32) -> (PlayerState, f32) {
        let mut p = Player::new(self);
        let rel = p.seek(self, t);
        (p.state, rel)
    }

    /// Truncates the trajectory at `time_sec`, splitting the segment there.
    /// The trajectory is unchanged on error.
    pub fn cut_at(&mut self, time_sec: f32) -> Result<()> {
        let (state, _) = self.segment_at(time_sec);
        let seg = &state.segment;
        let cut_msec: u32 = if time_sec * 1000.0 >= 4294967296.0 {
            u32::MAX
        } else if time_sec > 0.0 {
            roundf(time_sec * 1000.0) as u32
        } else {
            0
        };

        if state.length == 0 || cut_msec <= seg.start_time_msec {
            self.buf.truncate(state.start);
        } else if cut_msec >= seg.end_time_msec {
            self.buf.truncate(state.start + state.length);
        } else {
            let rel = (cut_msec - seg.start_time_msec) as f32 / seg.duration_msec as f32;
            let header = self.buf[state.start];
            let mut off = state.start + 3;
            let mut scaled = [[0i16; 8]; 3];
            let mut counts = [0usize; 3];
            let start = [seg.start.x, seg.start.y, seg.start.z];
            for axis in 0..3 {
                let n = num_coords(header >> (2 * axis));
                counts[axis] = n;
                let mut src = [0.0f32; 8];
                let mut dst = [0.0f32; 8];
                src[0] = start[axis];
                for s in src.iter_mut().take(n).skip(1) {
                    *s = self.coord(&mut off);
                }
                bezier_cut_at(&mut dst, &src[..n], rel)?;
                for i in 1..n {
                    scaled[axis][i] = scale_coordinate(self.scale, dst[i])?;
                }
            }
            let mut off = state.start + 1;
            write_u16(&mut self.buf, &mut off, (cut_msec - seg.start_time_msec) as u16);
            for axis in 0..3 {
                for i in 1..counts[axis] {
                    write_u16(&mut self.buf, &mut off, scaled[axis][i] as u16);
                }
            }
            self.buf.truncate(state.start + state.length);
        }
        Ok(())
    }
}

fn validate(buf: &[u8]) -> Result<()> {
    if buf.len() < HEADER_LENGTH {
        return Err(Error::Corrupted);
    }
    if buf[0] & 0x7f == 0 {
        return Ok(());
    }
    let mut off = HEADER_LENGTH;
    while off < buf.len() {
        if buf.len() - off < 3 {
            return Err(Error::Corrupted);
        }
        let h = buf[off];
        let n: usize = (0..4).map(|a| num_coords(h >> (2 * a)) - 1).sum();
        off += 3 + 2 * n;
        if off > buf.len() {
            return Err(Error::Corrupted);
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Segment {
    pub start_time_msec: u32,
    pub end_time_msec: u32,
    /// u16 in the format; u32 here so the final hold segment fits.
    pub duration_msec: u32,
    pub start_time_sec: f32,
    pub end_time_sec: f32,
    pub duration_sec: f32,
    pub start: Vec3,
    pub end: Vec3,
    offset: usize,
    flags: u8,
    poly: Poly3,
    dpoly: Poly3,
    ddpoly: Poly3,
}

const POLY_VALID: u8 = 1;
const DPOLY_VALID: u8 = 2;
const DDPOLY_VALID: u8 = 4;

impl Segment {
    pub fn poly(&mut self, traj: &Trajectory) -> &Poly3 {
        if self.flags & POLY_VALID == 0 {
            let buf = &traj.buf;
            let mut off = self.offset;
            let header = buf[off];
            off += 3;
            let start = [self.start.x, self.start.y, self.start.z];
            let mut polys = [Poly::default(); 3];
            let mut ends = [0.0f32; 3];
            for axis in 0..3 {
                let n = num_coords(header >> (2 * axis));
                let mut coords = [0.0f32; 8];
                coords[0] = start[axis];
                for c in coords.iter_mut().take(n).skip(1) {
                    *c = read_i16(buf, &mut off) as f32 * traj.scale;
                }
                ends[axis] = coords[n - 1];
                polys[axis] = Poly::bezier(1.0, &coords[..n]);
            }
            self.end = Vec3::new(ends[0], ends[1], ends[2]);
            self.poly = Poly3 { x: polys[0], y: polys[1], z: polys[2] };
            self.flags |= POLY_VALID;
        }
        &self.poly
    }

    pub fn dpoly(&mut self, traj: &Trajectory) -> &Poly3 {
        if self.flags & DPOLY_VALID == 0 {
            let mut p = *self.poly(traj);
            p.deriv();
            if self.duration_sec.abs() > 1.0e-6 {
                p.scale(1.0 / self.duration_sec);
            }
            self.dpoly = p;
            self.flags |= DPOLY_VALID;
        }
        &self.dpoly
    }

    pub fn ddpoly(&mut self, traj: &Trajectory) -> &Poly3 {
        if self.flags & DDPOLY_VALID == 0 {
            let mut p = *self.dpoly(traj);
            p.deriv();
            if self.duration_sec.abs() > 1.0e-6 {
                p.scale(1.0 / self.duration_sec);
            }
            self.ddpoly = p;
            self.flags |= DDPOLY_VALID;
        }
        &self.ddpoly
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlayerState {
    /// Offset and length of the current segment in the buffer; length 0
    /// means the final "hold forever" segment.
    pub start: usize,
    pub length: usize,
    pub segment: Segment,
}

/// Plays a trajectory. The trajectory is passed to each call so the player
/// can live next to it; call `rewind` after modifying the trajectory.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Player {
    pub state: PlayerState,
}

impl Player {
    pub fn new(traj: &Trajectory) -> Self {
        let mut p = Player::default();
        p.rewind(traj);
        p
    }

    pub fn rewind(&mut self, traj: &Trajectory) {
        self.build_segment(traj, traj.header_length, 0, traj.start);
    }

    pub fn build_next_segment(&mut self, traj: &Trajectory) {
        let s = &self.state;
        let (off, t, end) = (s.start + s.length, s.segment.end_time_msec, s.segment.end);
        self.build_segment(traj, off, t, end);
    }

    pub fn has_more_segments(&self) -> bool {
        self.state.length > 0
    }

    pub fn current_segment(&mut self) -> &mut Segment {
        &mut self.state.segment
    }

    fn build_segment(&mut self, traj: &Trajectory, mut off: usize, start_time_msec: u32, start: Vec3) {
        let mut seg = Segment {
            start,
            start_time_msec,
            start_time_sec: start_time_msec as f32 / 1000.0,
            offset: off,
            ..Default::default()
        };
        self.state.start = off;
        self.state.length = 0;

        if off >= traj.buf.len() || traj.scale == 0.0 {
            seg.poly = Poly3::constant(start);
            seg.duration_msec = u32::MAX - start_time_msec;
            seg.duration_sec = f32::INFINITY;
            seg.end_time_msec = u32::MAX;
            seg.end_time_sec = f32::INFINITY;
            seg.flags = POLY_VALID;
            seg.end = start;
            self.state.segment = seg;
            return;
        }

        let buf = &traj.buf;
        let header = buf[off];
        off += 1;
        seg.duration_msec = read_u16(buf, &mut off) as u32;
        seg.duration_sec = seg.duration_msec as f32 / 1000.0;
        seg.end_time_msec = start_time_msec.saturating_add(seg.duration_msec);
        seg.end_time_sec = seg.end_time_msec as f32 / 1000.0;

        let starts = [start.x, start.y, start.z];
        let mut ends = [0.0f32; 3];
        for axis in 0..3 {
            let n = num_coords(header >> (2 * axis));
            if n > 1 {
                off += 2 * (n - 2);
                ends[axis] = traj.coord(&mut off);
            } else {
                ends[axis] = starts[axis];
            }
        }
        off += 2 * (num_coords(header >> 6) - 1);
        seg.end = Vec3::new(ends[0], ends[1], ends[2]);
        self.state.length = off - self.state.start;
        self.state.segment = seg;
    }

    /// Moves to the segment containing `t` (rewinding if needed) and returns
    /// the relative time within it.
    pub fn seek(&mut self, traj: &Trajectory, t: f32) -> f32 {
        let t = if t <= 0.0 { 0.0 } else { t };
        loop {
            let seg = &self.state.segment;
            if seg.start_time_sec > t {
                self.rewind(traj);
            } else if seg.end_time_sec < t {
                self.build_next_segment(traj);
            } else if !t.is_finite() {
                return 1.0;
            } else if seg.duration_sec.abs() > 1.0e-6 {
                return (t - seg.start_time_sec) / seg.duration_sec;
            } else {
                return 0.5;
            }
        }
    }

    pub fn position_at(&mut self, traj: &Trajectory, t: f32) -> Vec3 {
        let rel = self.seek(traj, t);
        self.state.segment.poly(traj).eval(rel)
    }

    pub fn velocity_at(&mut self, traj: &Trajectory, t: f32) -> Vec3 {
        let rel = self.seek(traj, t);
        self.state.segment.dpoly(traj).eval(rel)
    }

    pub fn acceleration_at(&mut self, traj: &Trajectory, t: f32) -> Vec3 {
        let rel = self.seek(traj, t);
        self.state.segment.ddpoly(traj).eval(rel)
    }
}

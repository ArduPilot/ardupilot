// SPDX-License-Identifier: GPL-3.0-or-later
//! Nested Lua-style path composition; ordinary rolls do not rotate later segments.

use super::{
    math::*,
    primitives::{alignment_distance, Geometry, Roll},
};
use alloc::{string::String, vec::Vec};
use ap_rust_sdk::{enums::MavSeverity, Context, Error, Quaternion, Result, Vector3f};

#[derive(Clone, Copy)]
pub struct Evaluation {
    pub speed: f32,
    pub entry_rate: f32,
    pub announce: bool,
}

#[cfg(test)]
mod size_tests {
    use super::*;
    #[test]
    fn bounded_node_size() {
        assert!(core::mem::size_of::<Path>() <= 256);
        assert!(core::mem::size_of::<Segment>() <= 320);
    }
}

#[derive(Clone, Copy, Default)]
pub struct Attributes {
    pub roll_ref: Option<f32>,
    pub set_orient: Option<Quaternion>,
    pub thr_boost: Option<bool>,
    pub pos_corr: Option<f32>,
    pub pos_gain_mul: Option<f32>,
    pub shift_xy: Option<bool>,
    pub timestamp: Option<f32>,
}

pub struct Path {
    kind: Kind,
    pub attributes: Attributes,
    pub message: Option<String>,
    length: f32,
    final_orientation: Quaternion,
    extents: Option<(f32, f32)>,
    pub rebuild_on_reentry: bool,
    pub speed_dependent: Option<SpeedDependent>,
}

pub enum SpeedDependent {
    BarrelBank { radius: f32 },
}

pub fn barrel_bank(c: &Context<'_>, radius: f32, speed: f32) -> Result<f32> {
    Ok(deg(c
        .libm()
        .atan2f((speed * speed) / (radius * 9.80665), 1.0)?))
}

enum Kind {
    Leaf {
        geometry: Geometry,
        roll: Roll,
    },
    Align(Option<f32>),
    Composite {
        segments: Vec<Segment>,
        highest: usize,
        last_index: usize,
    },
}

struct Segment {
    path: Path,
    start_time: f32,
    start_pos: Vector3f,
    start_orientation: Quaternion,
    start_angle: f32,
    start_correction: f32,
}

impl Path {
    pub fn cache_words(&self) -> usize {
        match &self.kind {
            Kind::Composite { segments, .. } => {
                4 + segments.iter().map(|s| s.path.cache_words()).sum::<usize>()
            }
            _ => 0,
        }
    }

    // Preorder composite records: child count/flags, message cursor, child ordinal, barrel bank.
    pub fn save_cache(&self, words: &mut [u32], cursor: &mut usize) -> Result {
        if let Kind::Composite {
            segments,
            highest,
            last_index,
        } = &self.kind
        {
            if *cursor + 4 > words.len() {
                return Err(Error::NoMemory);
            }
            let bank = if self.speed_dependent.is_some() {
                match &segments[0].path.kind {
                    Kind::Leaf {
                        roll: Roll::EntryExit(a),
                        ..
                    } => -*a,
                    _ => return Err(Error::Invalid),
                }
            } else {
                0.0
            };
            words[*cursor..*cursor + 4].copy_from_slice(&[
                segments.len() as u32 | (u32::from(self.speed_dependent.is_some()) << 16),
                *highest as u32,
                *last_index as u32,
                bank.to_bits(),
            ]);
            *cursor += 4;
            for segment in segments {
                segment.path.save_cache(words, cursor)?;
            }
        }
        Ok(())
    }

    pub fn restore_cache(&mut self, words: &[u32], cursor: &mut usize) -> Result {
        if let Kind::Composite {
            segments,
            highest,
            last_index,
        } = &mut self.kind
        {
            if *cursor + 4 > words.len() {
                return Err(Error::Invalid);
            }
            let a = &words[*cursor..*cursor + 4];
            if a[0] != segments.len() as u32 | (u32::from(self.speed_dependent.is_some()) << 16)
                || a[1] as usize > segments.len()
                || a[2] as usize >= segments.len()
                || !f32::from_bits(a[3]).is_finite()
            {
                return Err(Error::Invalid);
            }
            *highest = a[1] as usize;
            *last_index = a[2] as usize;
            if self.speed_dependent.is_some() {
                if segments.len() != 3 {
                    return Err(Error::Invalid);
                }
                let bank = f32::from_bits(a[3]);
                for (index, angle) in [(0, -bank), (2, bank)] {
                    match &mut segments[index].path.kind {
                        Kind::Leaf { roll, .. } => *roll = Roll::EntryExit(angle),
                        _ => return Err(Error::Invalid),
                    }
                }
            }
            *cursor += 4;
            for segment in segments {
                segment.path.restore_cache(words, cursor)?;
            }
        }
        Ok(())
    }

    pub fn restart(&mut self) {
        // Lua resets only the root message cursor; child cache misses perform
        // their normal re-entry refresh when the first segment is sampled.
        if let Kind::Composite { highest, .. } = &mut self.kind {
            *highest = 0;
        }
    }

    pub fn leaf(c: &Context<'_>, geometry: Geometry, roll: Roll) -> Result<Self> {
        Ok(Self {
            length: geometry.length(c)?,
            final_orientation: geometry.final_orientation(c)?,
            kind: Kind::Leaf { geometry, roll },
            attributes: Attributes::default(),
            message: None,
            extents: None,
            rebuild_on_reentry: false,
            speed_dependent: None,
        })
    }

    pub fn align(alignment: Option<f32>) -> Self {
        Self {
            kind: Kind::Align(alignment),
            length: 0.0,
            final_orientation: Quaternion::default(),
            attributes: Attributes::default(),
            message: None,
            extents: None,
            rebuild_on_reentry: false,
            speed_dependent: None,
        }
    }

    pub fn compose(
        c: &Context<'_>,
        mut paths: Vec<Path>,
        box_width: f32,
        eval: Evaluation,
    ) -> Result<Self> {
        if paths.is_empty()
            || !eval.speed.is_finite()
            || eval.speed <= 0.0
            || !eval.entry_rate.is_finite()
            || eval.entry_rate <= 0.0
            || !box_width.is_finite()
        {
            return Err(Error::Invalid);
        }
        let mut segments = Vec::new();
        segments
            .try_reserve_exact(paths.len())
            .map_err(|_| Error::NoMemory)?;
        let mut orientation = Quaternion::default();
        let mut pos = Vector3f::default();
        let (mut angle, mut correction, mut length) = (0.0, 0.0, 0.0);
        // Resolve alignments while the following maneuver still exists. The
        // copied position/orientation are those at the alignment's start.
        for i in 0..paths.len() {
            if let Kind::Align(alignment) = paths[i].kind {
                if i + 1 == paths.len() {
                    return Err(Error::Invalid);
                }
                let extents = paths[i + 1].extents(c, eval)?;
                let d = alignment_distance(c, alignment, box_width, extents, pos, orientation)?;
                paths[i].kind = Kind::Leaf {
                    geometry: Geometry::Straight(d),
                    roll: Roll::Angle(0.0),
                };
                paths[i].length = d;
                paths[i].rebuild_on_reentry = false;
            }
            let child = &mut paths[i];
            // Zero-length components occur in valid Lua schedules. They are
            // skipped at evaluation boundaries, as by get_subpath_t().
            if !child.length.is_finite() || child.length < 0.0 {
                return Err(Error::Invalid);
            }
            segments.push((pos, orientation, angle, correction));
            length += child.length;
            pos = pos.add(
                c,
                earth_to_body(c, orientation, child.position(c, 1.0, eval)?)?,
            )?;
            orientation = child.final_orientation.mul(c, orientation)?;
            orientation.normalize(c)?;
            angle += child.roll(c, 1.0, child.length / eval.speed, eval)?;
            correction += child.correction(c, 1.0, eval)?;
            if let Some(q) = child.attributes.set_orient {
                orientation = q;
            }
            if let Some(roll) = child.attributes.roll_ref {
                let mut q = Quaternion::default();
                q.from_axis_angle(c, Vector3f::new(1.0, 0.0, 0.0), rad(roll))?;
                orientation = orientation.mul(c, q)?;
                orientation.normalize(c)?;
            }
        }
        if !length.is_finite() || length <= 0.0 {
            return Err(Error::Invalid);
        }
        if paths.last().unwrap().length <= 0.0 {
            return Err(Error::Invalid);
        }
        let mut q = Quaternion::default();
        q.from_axis_angle(c, Vector3f::new(1.0, 0.0, 0.0), rad(wrap_180(c, angle)?))?;
        let final_orientation = q.mul(c, orientation)?;
        let mut result = Vec::new();
        result
            .try_reserve_exact(paths.len())
            .map_err(|_| Error::NoMemory)?;
        let mut time = 0.0;
        for (path, (start_pos, start_orientation, start_angle, start_correction)) in
            paths.into_iter().zip(segments)
        {
            let start_time = time;
            time += path.length / length;
            result.push(Segment {
                path,
                start_time,
                start_pos,
                start_orientation,
                start_angle,
                start_correction,
            });
        }
        Ok(Self {
            kind: Kind::Composite {
                last_index: result.len() - 1,
                segments: result,
                highest: 0,
            },
            length,
            final_orientation,
            attributes: Attributes::default(),
            message: None,
            extents: None,
            rebuild_on_reentry: false,
            speed_dependent: None,
        })
    }

    pub fn length(&self) -> f32 {
        self.length
    }
    #[allow(dead_code, reason = "Path-composition oracle API")]
    pub fn final_orientation(&self) -> Quaternion {
        self.final_orientation
    }

    fn segment<'a>(
        c: &Context<'_>,
        segments: &'a mut [Segment],
        highest: &mut usize,
        last_index: &mut usize,
        length: f32,
        t: f32,
        eval: Evaluation,
    ) -> Result<(&'a mut Segment, f32)> {
        if !eval.speed.is_finite()
            || eval.speed <= 0.0
            || !eval.entry_rate.is_finite()
            || eval.entry_rate <= 0.0
        {
            return Err(Error::Invalid);
        }
        let mut i = 0;
        while i + 1 < segments.len()
            && t >= segments[i].start_time + segments[i].path.length / length
        {
            i += 1;
        }
        let segment = &mut segments[i];
        if *last_index != i {
            if segment.path.rebuild_on_reentry {
                segment.path.activate(c, eval.speed)?;
            }
            *last_index = i;
        }
        let sub_t = (t - segment.start_time) / (segment.path.length / length);
        if i + 1 > *highest && t > 0.0 && t < 1.0 {
            if eval.announce {
                if let Some(message) = &segment.path.message {
                    // Telemetry is best effort, but stop must still unwind.
                    if c.gcs().send_text(MavSeverity(1), message) == Err(Error::Stopped) {
                        return Err(Error::Stopped);
                    }
                }
            }
            *highest = i + 1;
        }
        Ok((segment, sub_t))
    }

    pub fn activate(&mut self, c: &Context<'_>, speed: f32) -> Result {
        // Lua recreates function-backed paths on every single-slot cache miss.
        // AirShow's only speed-dependent construction is barrel-roll bank;
        // its geometry/length and final roll are speed independent. Refresh
        // those banks without allocating/rebuilding the retained geometry.
        let bank = match self.speed_dependent {
            Some(SpeedDependent::BarrelBank { radius }) => Some(barrel_bank(c, radius, speed)?),
            None => None,
        };
        if let Kind::Composite {
            segments,
            highest,
            last_index,
        } = &mut self.kind
        {
            *highest = 0;
            *last_index = segments.len() - 1;
            if let Some(bank) = bank {
                if segments.len() != 3 {
                    return Err(Error::Invalid);
                }
                for (index, value) in [(0, -bank), (2, bank)] {
                    if let Kind::Leaf { roll, .. } = &mut segments[index].path.kind {
                        *roll = Roll::EntryExit(value);
                    } else {
                        return Err(Error::Invalid);
                    }
                }
            }
        }
        Ok(())
    }

    pub fn position(&mut self, c: &Context<'_>, t: f32, eval: Evaluation) -> Result<Vector3f> {
        match &mut self.kind {
            Kind::Leaf { geometry, .. } => geometry.position(c, t),
            Kind::Align(_) => Err(Error::Invalid),
            Kind::Composite {
                segments,
                highest,
                last_index,
            } => {
                let (s, u) = Self::segment(c, segments, highest, last_index, self.length, t, eval)?;
                earth_to_body(c, s.start_orientation, s.path.position(c, u, eval)?)?
                    .add(c, s.start_pos)
            }
        }
    }

    pub fn roll(
        &mut self,
        c: &Context<'_>,
        t: f32,
        duration: f32,
        eval: Evaluation,
    ) -> Result<f32> {
        match &mut self.kind {
            Kind::Leaf { roll, .. } => wrap_180(c, roll.get(t, duration, eval.entry_rate)),
            Kind::Align(_) => Err(Error::Invalid),
            Kind::Composite {
                segments,
                highest,
                last_index,
            } => {
                let (s, u) = Self::segment(c, segments, highest, last_index, self.length, t, eval)?;
                Ok(s.path.roll(c, u, s.path.length / eval.speed, eval)? + s.start_angle)
            }
        }
    }

    pub fn correction(&mut self, c: &Context<'_>, t: f32, eval: Evaluation) -> Result<f32> {
        match &mut self.kind {
            Kind::Leaf { geometry, .. } => geometry.correction(c, t),
            Kind::Align(_) => Err(Error::Invalid),
            Kind::Composite {
                segments,
                highest,
                last_index,
            } => {
                let (s, u) = Self::segment(c, segments, highest, last_index, self.length, t, eval)?;
                Ok(s.path.correction(c, u, eval)? + s.start_correction)
            }
        }
    }

    pub fn extents(&mut self, c: &Context<'_>, eval: Evaluation) -> Result<(f32, f32)> {
        if let Some(extents) = self.extents {
            return Ok(extents);
        }
        let x = self.position(c, 0.0, eval)?.x;
        let (mut low, mut high, mut t) = (x, x, 0.0);
        while t <= 1.0 {
            let x = self.position(c, t, eval)?.x;
            low = min(low, x);
            high = max(high, x);
            t += 0.02;
        }
        self.extents = Some((low, high));
        Ok((low, high))
    }

    pub fn attributes(&mut self, c: &Context<'_>, t: f32, eval: Evaluation) -> Result<Attributes> {
        match &mut self.kind {
            Kind::Composite {
                segments,
                highest,
                last_index,
            } => {
                let (s, u) = Self::segment(c, segments, highest, last_index, self.length, t, eval)?;
                let outer = s.path.attributes;
                let inner = s.path.attributes(c, u, eval)?;
                Ok(Attributes {
                    roll_ref: outer.roll_ref.or(inner.roll_ref),
                    set_orient: outer.set_orient.or(inner.set_orient),
                    thr_boost: outer.thr_boost.filter(|v| *v).or(inner.thr_boost),
                    pos_corr: outer.pos_corr.or(inner.pos_corr),
                    pos_gain_mul: outer.pos_gain_mul.or(inner.pos_gain_mul),
                    shift_xy: outer.shift_xy.filter(|v| *v).or(inner.shift_xy),
                    timestamp: outer.timestamp.or(inner.timestamp),
                })
            }
            _ => Ok(self.attributes),
        }
    }

    #[allow(dead_code, reason = "Path-composition oracle API")]
    pub fn next_segment_start(&mut self, c: &Context<'_>, t: f32, eval: Evaluation) -> Result<f32> {
        match &mut self.kind {
            Kind::Composite {
                segments,
                highest,
                last_index,
            } => {
                let (s, u) = Self::segment(c, segments, highest, last_index, self.length, t, eval)?;
                let fraction = s.path.length / self.length;
                let end = s.start_time + fraction;
                if matches!(&s.path.kind, Kind::Composite { .. }) {
                    Ok(
                        s.start_time
                            + s.path.next_segment_start(c, u, eval)? * (end - s.start_time),
                    )
                } else {
                    Ok(end)
                }
            }
            _ => Ok(1.0),
        }
    }
}

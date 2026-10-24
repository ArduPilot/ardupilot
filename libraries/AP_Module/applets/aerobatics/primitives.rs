// SPDX-License-Identifier: GPL-3.0-or-later
//! Direct port of the Lua aerobatics roll and geometric path components.

use super::math::*;
use alloc::vec::Vec;
use ap_rust_sdk::{Context, Error, Quaternion, Result, Vector3f};

#[derive(Debug)]
pub enum Roll {
    Angle(f32),
    EntryExit(f32),
    #[allow(dead_code, reason = "Lua primitive oracle coverage")]
    Entry(f32),
    #[allow(dead_code, reason = "Lua primitive oracle coverage")]
    Exit(f32),
    Sequence(SequenceRoll),
}

#[derive(Debug)]
pub struct SequenceRoll {
    segments: Vec<RollSegment>,
    total: f32,
}

#[derive(Debug)]
struct RollSegment {
    weight: f32,
    angle: f32,
    start_t: f32,
    end_t: f32,
    start_angle: f32,
}

impl Roll {
    pub fn sequence(values: &[(f32, f32)]) -> Result<Self> {
        if values.is_empty() {
            return Err(Error::Invalid);
        }
        let total = values.iter().fold(0.0, |a, v| a + v.0);
        let mut segments = Vec::new();
        segments
            .try_reserve_exact(values.len())
            .map_err(|_| Error::NoMemory)?;
        let (mut t, mut angle) = (0.0, 0.0);
        for &(weight, change) in values {
            let start_t = t;
            t += weight / total;
            segments.push(RollSegment {
                weight,
                angle: change,
                start_t,
                end_t: t,
                start_angle: angle,
            });
            angle += change;
        }
        Ok(Self::Sequence(SequenceRoll { segments, total }))
    }

    pub fn get(&self, t: f32, time_s: f32, entry_rate: f32) -> f32 {
        match self {
            Self::Angle(angle) => {
                if *angle == 0.0 {
                    0.0
                } else {
                    angle * t
                }
            }
            Self::EntryExit(angle) => {
                let entry_t = min((angle.abs() / entry_rate) / time_s, 0.5);
                if t <= 0.0 {
                    return 0.0;
                }
                if t < entry_t {
                    return angle * t / entry_t;
                }
                if t < 1.0 - entry_t {
                    return *angle;
                }
                if *angle == 0.0 || t >= 1.0 {
                    return 0.0;
                }
                (1.0 - (t - (1.0 - entry_t)) / entry_t) * angle
            }
            Self::Entry(angle) => {
                let entry_t = min((angle.abs() / entry_rate) / time_s, 0.5);
                if t < entry_t {
                    angle * t / entry_t
                } else {
                    *angle
                }
            }
            Self::Exit(angle) => {
                let entry_t = (angle.abs() / entry_rate) / time_s;
                if t < 1.0 - entry_t || *angle == 0.0 {
                    return 0.0;
                }
                ((t - (1.0 - entry_t)) / entry_t) * angle
            }
            Self::Sequence(SequenceRoll { segments, total }) => {
                for s in segments {
                    if t <= s.end_t {
                        let t2 = (t - s.start_t) / (s.weight / total);
                        return s.start_angle + t2 * s.angle;
                    }
                }
                let last = &segments[segments.len() - 1];
                last.start_angle + last.angle
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Geometry {
    Straight(f32),
    #[allow(dead_code, reason = "Lua primitive oracle coverage")]
    Reverse(f32),
    Vertical {
        radius: f32,
        angle: f32,
    },
    Horizontal {
        radius: f32,
        angle: f32,
        height: f32,
    },
    Cylinder {
        radius: f32,
        distance: f32,
        spirals: f32,
        gamma: f32,
    },
}

impl Geometry {
    pub fn cylinder(c: &Context<'_>, radius: f32, distance: f32, spirals: f32) -> Result<Self> {
        let gamma = c.libm().atan2f((distance / spirals) / (2.0 * PI * radius.abs()), 1.0)?;
        Ok(Self::Cylinder {
            radius,
            distance,
            spirals,
            gamma,
        })
    }

    pub fn position(&self, c: &Context<'_>, t: f32) -> Result<Vector3f> {
        Ok(match *self {
            Self::Straight(d) => Vector3f::new(d * t, 0.0, 0.0),
            Self::Reverse(d) => Vector3f::new(
                if t < 0.5 {
                    d * t * 2.0
                } else {
                    d - d * (t - 0.5) * 2.0
                },
                0.0,
                0.0,
            ),
            Self::Vertical { radius, angle } => {
                let a = wrap_2pi(c, t * rad(angle))?;
                Vector3f::new(
                    radius.abs() * c.libm().sinf(a)?,
                    0.0,
                    -radius * (1.0 - c.libm().cosf(a)?),
                )
            }
            Self::Horizontal {
                radius,
                angle,
                height,
            } => {
                let a = t * rad(angle);
                Vector3f::new(
                    radius.abs() * c.libm().sinf(a)?,
                    radius * (1.0 - c.libm().cosf(a)?),
                    -height * t,
                )
            }
            Self::Cylinder {
                radius,
                distance,
                spirals,
                gamma,
            } => {
                let a = t * spirals * PI * 2.0;
                let v = Vector3f::new(
                    distance * t,
                    radius.abs() * c.libm().sinf(a + PI)?,
                    -radius.abs() * (1.0 - c.libm().cosf(a)?),
                );
                let mut q = Quaternion::default();
                q.from_axis_angle(c, Vector3f::new(0.0, 0.0, 1.0), 0.5 * PI - gamma)?;
                let mut v = earth_to_body(c, q, v)?;
                if radius < 0.0 {
                    v.y = -v.y;
                }
                v
            }
        })
    }

    pub fn length(&self, c: &Context<'_>) -> Result<f32> {
        Ok(match *self {
            Self::Straight(d) => d,
            Self::Reverse(d) => d * 2.0,
            Self::Vertical { radius, angle } => radius.abs() * 2.0 * PI * angle.abs() / 360.0,
            Self::Horizontal {
                radius,
                angle,
                height,
            } => {
                let circumference = 2.0 * PI * radius.abs();
                let h = height * 360.0 / angle.abs();
                c.libm().sqrtf(h * h + circumference * circumference)? * angle.abs() / 360.0
            }
            Self::Cylinder {
                radius,
                distance,
                spirals,
                ..
            } => {
                let circumference = 2.0 * PI * radius.abs();
                let d = distance / spirals;
                c.libm().sqrtf(d * d + circumference * circumference)? * spirals
            }
        })
    }

    pub fn final_orientation(&self, c: &Context<'_>) -> Result<Quaternion> {
        let mut q = Quaternion::default();
        match *self {
            Self::Vertical { radius, angle } => {
                q.from_axis_angle(
                    c,
                    Vector3f::new(0.0, 1.0, 0.0),
                    sgn(radius) * rad(wrap_180(c, angle)?),
                )?;
                q.normalize(c)?;
            }
            Self::Horizontal { radius, angle, .. } => {
                q.from_axis_angle(c, Vector3f::new(0.0, 0.0, 1.0), sgn(radius) * rad(angle))?;
            }
            _ => (),
        }
        Ok(q)
    }

    pub fn correction(&self, c: &Context<'_>, t: f32) -> Result<f32> {
        Ok(match *self {
            Self::Horizontal {
                radius,
                angle,
                height,
            } if height != 0.0 => {
                let gamma = c.libm().atan2f(height * (360.0 / angle) / (2.0 * PI * radius), 1.0)?;
                -t * angle * c.libm().sinf(gamma)?
            }
            Self::Cylinder {
                radius,
                spirals,
                gamma,
                ..
            } => sgn(radius) * t * 360.0 * c.libm().sinf(gamma)? * spirals,
            _ => 0.0,
        })
    }

    #[allow(dead_code, reason = "Lua primitive oracle coverage")]
    pub fn extents(&self, c: &Context<'_>) -> Result<(f32, f32)> {
        let x = self.position(c, 0.0)?.x;
        let (mut low, mut high) = (x, x);
        // Lua's floating numeric-for accumulates the step rather than i/50.
        let mut t = 0.0;
        while t <= 1.0 {
            let x = self.position(c, t)?.x;
            low = min(low, x);
            high = max(high, x);
            t += 0.02;
        }
        Ok((low, high))
    }
}

pub fn alignment_distance(
    c: &Context<'_>,
    alignment: Option<f32>,
    box_width: f32,
    extents: (f32, f32),
    start: Vector3f,
    orientation: Quaternion,
) -> Result<f32> {
    let reverse = deg(orientation.get_euler_yaw(c)?).abs() > 90.0;
    let start_x = if reverse { start.x } else { -start.x };
    let distance = match alignment {
        Some(a) => (box_width / 2.0) * a + start_x - extents.1,
        None => start_x - (extents.0 + extents.1) * 0.5,
    };
    Ok(max(distance, 0.01))
}

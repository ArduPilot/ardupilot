// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]
#![allow(dead_code)]

extern crate alloc;
#[path = "../../applets/aerobatics/math.rs"]
mod math;
#[path = "../../applets/aerobatics/primitives.rs"]
mod primitives;
use ap_rust_sdk::{Context, Error, LogValue as V, Result, Vector3f};
use primitives::{Geometry as G, Roll};
ap_rust_sdk::module!(run);

struct Oracle {
    offset: u32,
    cache: [u8; 256],
    available: usize,
}
impl Oracle {
    fn expected(&mut self, c: &Context<'_>) -> Result<f32> {
        let index = self.offset as usize % self.cache.len();
        if index == 0 {
            self.available = c
                .file().read_at("aerobatics-golden.bin", self.offset, &mut self.cache)?
                .ok_or(Error::Invalid)?;
        }
        if index + 4 > self.available {
            return Err(Error::Invalid);
        }
        self.offset += 4;
        Ok(f32::from_le_bytes(
            self.cache[index..index + 4].try_into().unwrap(),
        ))
    }

    fn record(&mut self, c: &Context<'_>, name: &str, labels: &str, values: &[V<'_>]) -> Result {
        if !cfg!(target_os = "none") {
            return c.logger().write(name, labels, None, values);
        }
        // H743 consumes values captured from the original Lua implementation
        // in SITL, through real FATFS. No Rust-produced expected values.
        for value in values {
            let actual = match value {
                V::U8(v) => *v as f32,
                V::Float(v) => *v,
                _ => return Err(Error::Invalid),
            };
            let expected = self.expected(c)?;
            if actual == expected || (actual.is_nan() && expected.is_nan()) {
                continue;
            }
            if !actual.is_finite()
                || !expected.is_finite()
                || (actual - expected).abs() > math::max(2e-5, expected.abs() * 2e-6)
            {
                return Err(Error::Invalid);
            }
        }
        Ok(())
    }
}

fn helpers(c: &Context<'_>, oracle: &mut Oracle) -> Result {
    let x = Vector3f::new(1.0, 0.0, 0.0);
    for b in [
        Vector3f::default(),
        Vector3f::new(-1.0, 0.0, 0.0),
        Vector3f::new(-1.0, 1e-8, 0.0),
        Vector3f::new(f32::NAN, 0.0, 0.0),
    ] {
        if math::vectors_to_rotation(c, x, b) != Err(Error::Invalid) {
            return Err(Error::Host);
        }
    }
    for dt in [0.0, -1.0, f32::NAN, f32::INFINITY, 1e-40] {
        if math::tangents_to_rate(c, x, x, dt) != Err(Error::Invalid) {
            return Err(Error::Host);
        }
    }
    for (id, a) in [
        -540.0,
        -360.0,
        -180.0,
        -0.0,
        0.0,
        180.0,
        360.0,
        540.0,
        180.00002,
        1000000.0,
        f32::NAN,
        f32::INFINITY,
        -f32::INFINITY,
    ]
    .iter()
    .enumerate()
    {
        oracle.record(
            c,
            "ARWM",
            "Id,In,D,R,W360,W180,Wpi,W2pi,Sign,Min,Max,Con,Alpha",
            &[
                V::U8(id as u8),
                V::Float(*a),
                V::Float(math::deg(*a)),
                V::Float(math::rad(*a)),
                V::Float(math::wrap_360(c, *a)?),
                V::Float(math::wrap_180(c, *a)?),
                V::Float(math::wrap_pi(c, *a)?),
                V::Float(math::wrap_2pi(c, *a)?),
                V::Float(math::sgn(*a)),
                V::Float(math::min(*a, 0.0)),
                V::Float(math::max(*a, 0.0)),
                V::Float(math::constrain(*a, -90.0, 90.0)),
                V::Float(math::lowpass_alpha(0.025, 0.1)),
            ],
        )?;
    }
    for (id, angles) in [(0.0, 0.0, 0.0), (30.0, 45.0, -90.0), (-170.0, -80.0, 179.0)]
        .iter()
        .enumerate()
    {
        let q = math::orient(c, angles.0, angles.1, angles.2)?;
        let v = Vector3f::new(1.2, -2.3, 3.4);
        let r = math::earth_to_body(c, q, v)?;
        let b = math::body_to_earth(c, q, v)?;
        oracle.record(
            c,
            "ARQM",
            "Id,Q1,Q2,Q3,Q4,RX,RY,RZ,BX,BY,BZ,DCM",
            &[
                V::U8(id as u8),
                V::Float(q.q1),
                V::Float(q.q2),
                V::Float(q.q3),
                V::Float(q.q4),
                V::Float(r.x),
                V::Float(r.y),
                V::Float(r.z),
                V::Float(b.x),
                V::Float(b.y),
                V::Float(b.z),
                V::Float(math::dcm_c_y(q)),
            ],
        )?;
    }
    for (id, b) in [
        Vector3f::new(1.0, 0.0, 0.0),
        Vector3f::new(0.0, 1.0, 0.0),
        Vector3f::new(-1.0, 0.01, 0.0),
        Vector3f::new(1.0, 2.0, 3.0),
    ]
    .iter()
    .enumerate()
    {
        let a = Vector3f::new(1.0, 0.0, 0.0);
        let q = math::vectors_to_rotation(c, a, *b)?;
        let rate = math::tangents_to_rate(c, a, *b, 0.025)?;
        oracle.record(
            c,
            "ARVM",
            "Id,Q1,Q2,Q3,Q4,X,Y,Z",
            &[
                V::U8(id as u8),
                V::Float(q.q1),
                V::Float(q.q2),
                V::Float(q.q3),
                V::Float(q.q4),
                V::Float(rate.x),
                V::Float(rate.y),
                V::Float(rate.z),
            ],
        )?;
    }
    Ok(())
}

fn pause(c: &Context<'_>, ms: u32) -> Result {
    if cfg!(target_os = "none") {
        Ok(())
    } else {
        c.sleep_ms(ms)
    }
}

fn run(c: &Context<'_>) -> Result {
    pause(c, 2000)?;
    let mut oracle = Oracle {
        offset: 0,
        cache: [0; 256],
        available: 0,
    };
    let paths = [
        G::Straight(80.0),
        G::Straight(-2.5),
        G::Reverse(60.0),
        G::Vertical {
            radius: 25.0,
            angle: 360.0,
        },
        G::Vertical {
            radius: -40.0,
            angle: 225.0,
        },
        G::Vertical {
            radius: 20.5,
            angle: -540.0,
        },
        G::Vertical {
            radius: 0.0,
            angle: 90.0,
        },
        G::Horizontal {
            radius: -75.0,
            angle: 360.0,
            height: 0.0,
        },
        G::Horizontal {
            radius: 30.5,
            angle: 225.0,
            height: 20.0,
        },
        G::Horizontal {
            radius: -30.5,
            angle: -270.0,
            height: -15.0,
        },
        G::cylinder(c, 30.0, 100.0, 2.0)?,
        G::cylinder(c, -30.0, 100.0, 2.0)?,
        G::cylinder(c, 25.0, 60.0, 0.5)?,
    ];
    let times = [
        -0.1, 0.0, 0.02, 0.1, 0.25, 0.4999, 0.5, 0.5001, 0.75, 0.9, 1.0, 1.1,
    ];
    for (id, path) in paths.iter().enumerate() {
        let length = path.length(c)?;
        let q = path.final_orientation(c)?;
        let (low, high) = path.extents(c)?;
        for (i, &t) in times.iter().enumerate() {
            let p = path.position(c, t)?;
            oracle.record(
                c,
                "ARPG",
                "Id,I,T,X,Y,Z,Len,Q1,Q2,Q3,Q4,Cor,Lo,Hi",
                &[
                    V::U8(id as u8),
                    V::U8(i as u8),
                    V::Float(t),
                    V::Float(p.x),
                    V::Float(p.y),
                    V::Float(p.z),
                    V::Float(length),
                    V::Float(q.q1),
                    V::Float(q.q2),
                    V::Float(q.q3),
                    V::Float(q.q4),
                    V::Float(path.correction(c, t)?),
                    V::Float(low),
                    V::Float(high),
                ],
            )?;
            pause(c, 5)?;
        }
    }
    let rolls = [
        Roll::Angle(720.0),
        Roll::Angle(0.0),
        Roll::EntryExit(90.0),
        Roll::EntryExit(-90.0),
        Roll::EntryExit(0.0),
        Roll::Entry(90.0),
        Roll::Entry(0.0),
        Roll::Exit(-90.0),
        Roll::Exit(0.0),
        Roll::sequence(&[(1.0, 0.0), (2.0, 180.0), (1.0, 0.0), (3.0, -360.0)])?,
    ];
    for (id, roll) in rolls.iter().enumerate() {
        for (j, duration) in [0.5, 2.0, 10.0].iter().enumerate() {
            for (i, &t) in times.iter().enumerate() {
                let angle = roll.get(t, *duration, 60.0);
                oracle.record(
                    c,
                    "ARRL",
                    "Id,J,I,Ang,Wrap",
                    &[
                        V::U8(id as u8),
                        V::U8(j as u8),
                        V::U8(i as u8),
                        V::Float(angle),
                        V::Float(math::wrap_180(c, angle)?),
                    ],
                )?;
                pause(c, 5)?;
            }
        }
    }
    for (id, yaw) in [0.0, 90.0, 90.1, 180.0, -90.1].iter().enumerate() {
        for (j, alignment) in [None, Some(1.0), Some(-1.0)].iter().enumerate() {
            let d = primitives::alignment_distance(
                c,
                *alignment,
                400.0,
                (-30.0, 40.0),
                Vector3f::new(15.0, 0.0, 0.0),
                math::orient(c, 0.0, 0.0, *yaw)?,
            )?;
            oracle.record(
                c,
                "ARAL",
                "Id,J,D",
                &[V::U8(id as u8), V::U8(j as u8), V::Float(d)],
            )?;
        }
    }
    helpers(c, &mut oracle)?;
    if cfg!(target_os = "none") {
        if c.file().read_at("aerobatics-golden.bin", oracle.offset, &mut [0u8; 1])? != Some(0) {
            return Err(Error::Invalid);
        }
        c.text(6, "MATH ok")
    } else {
        c.text(6, "AEROP primitives done")
    }
}

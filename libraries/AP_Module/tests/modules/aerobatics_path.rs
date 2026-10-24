// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]
#![allow(dead_code)]
extern crate alloc;
#[path = "../../applets/aerobatics/maneuvers.rs"]
mod maneuvers;
#[path = "../../applets/aerobatics/math.rs"]
mod math;
#[path = "../../applets/aerobatics/path.rs"]
mod path;
#[path = "../../applets/aerobatics/primitives.rs"]
mod primitives;
#[path = "../../applets/aerobatics/schedule.rs"]
mod schedule;
use ap_rust_sdk::{Context, Error, LogValue as V, Result};
mod aerobatics_oracle;
ap_rust_sdk::module!(run);

fn run(c: &Context<'_>) -> Result {
    if !cfg!(target_os = "none") {
        c.sleep_ms(2000)?;
    }
    let mut oracle = aerobatics_oracle::Oracle::new();
    let schedule = schedule::Schedule::load(c, "trick72.txt")?.ok_or(Error::Invalid)?;
    let mut eval = path::Evaluation {
        speed: 18.0,
        entry_rate: 60.0,
        announce: !cfg!(target_os = "none"),
    };
    let mut path = schedule.build(c, 400.0, eval)?;
    if option_env!("AP_MODULE_AERO_BUILD_ONLY").is_some() {
        return c.text(6, "AEROP build done");
    }
    let length = path.length();
    let q = path.final_orientation();
    for i in 0..=500 {
        let t = if i > 0 && i % 7 == 0 && i < 475 {
            (i as f32 / 500.0) + 0.05
        } else {
            i as f32 / 500.0
        };
        eval.speed = 18.0 + (i % 11) as f32;
        eval.entry_rate = 60.0 + (i % 9) as f32;
        let p = path.position(c, t, eval)?;
        let roll = path.roll(c, t, length / eval.speed, eval)?;
        let correction = path.correction(c, t, eval)?;
        let next = path.next_segment_start(c, t, eval)?;
        let boost = path.attributes(c, t, eval)?.thr_boost.unwrap_or(false);
        oracle.record(
            c,
            "ARCP",
            "Id,T,X,Y,Z,Roll,Cor,Len,Q1,Q2,Q3,Q4,Next,Boost",
            &[
                V::U16(i),
                V::Float(t),
                V::Float(p.x),
                V::Float(p.y),
                V::Float(p.z),
                V::Float(roll),
                V::Float(correction),
                V::Float(length),
                V::Float(q.q1),
                V::Float(q.q2),
                V::Float(q.q3),
                V::Float(q.q4),
                V::Float(next),
                V::U8(boost as u8),
            ],
        )?;
        if !cfg!(target_os = "none") {
            c.sleep_ms(5)?;
        }
    }
    oracle.finish(c)?;
    c.text(
        6,
        if cfg!(target_os = "none") {
            "MATH ok"
        } else {
            "AEROP composition done"
        },
    )
}

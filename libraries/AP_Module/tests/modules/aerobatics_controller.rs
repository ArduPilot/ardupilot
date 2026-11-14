// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]
#![allow(dead_code)]
extern crate alloc;
#[path = "../../applets/aerobatics/controller.rs"]
mod controller;
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
use ap_rust_sdk::{Context, Error, LogValue as L, Quaternion, Result, Vector3f};
use controller::{Configuration, Controller, Input, Step};
mod aerobatics_oracle;
ap_rust_sdk::module!(run);
const VARIANT: Option<&str> = option_env!("AP_MODULE_AERO_CONTROLLER_VARIANT");

fn input(c: &Context<'_>, i: u16) -> Result<Input> {
    let n = i as f32;
    let mut attitude = Quaternion::default();
    attitude.from_euler(
        c,
        c.libm().sinf(n * 0.03)? * 0.2,
        c.libm().cosf(n * 0.01)? * 0.1,
        0.3,
    )?;
    let mut result = Input {
        now: (10000 + u32::from(i) * 25) as f32 * 0.001,
        pos: Vector3f::new(
            n * 0.45,
            c.libm().sinf(n * 0.01)? * 2.0,
            -30.0 + c.libm().cosf(n * 0.02)?,
        ),
        gyro: Vector3f::new(0.1, 0.2, 0.3),
        velocity: Vector3f::new(18.0, 0.02, -0.01),
        airspeed: 18.0,
        eas2tas: 1.0,
        attitude,
        throttle: 40.0,
    };
    if VARIANT.is_some() {
        result.velocity.x = if i % 37 == 0 {
            -5.0
        } else {
            10.0 + (i % 21) as f32
        };
        result.airspeed = 4.0 + (i % 37) as f32;
        result.eas2tas = if i % 43 == 0 { 5.0 } else { 1.0 };
        result.gyro.x = c.libm().sinf(n * 0.02)?;
    }
    Ok(result)
}

fn run(c: &Context<'_>) -> Result {
    if !cfg!(target_os = "none") {
        c.sleep_ms(2000)?;
    }
    let mut oracle = aerobatics_oracle::Oracle::controller();
    let mut p = Configuration {
        ang_accel: 6000.0,
        ang_tc: 0.1,
        thr_pit_ff: 80.0,
        spd_p: 5.0,
        spd_i: 25.0,
        roll_corr_tc: 0.25,
        time_corr_p: 1.0,
        err_corr_p: 2.0,
        err_corr_d: 2.8,
        entry_rate: 60.0,
        thr_lookahead: 1.0,
        thr_min: 0.0,
        thr_boost: 50.0,
        yaw_accel: 1500.0,
        lookahead: 0.5,
        path_scale: 1.0,
        ke_rudd: 25.0,
        ke_rudd_lookahead: 0.25,
        alt_abort: 15.0,
        airspeed_min: 9.0,
        scaling_speed: 15.0,
        airspeed_cruise: 18.0,
    };
    if VARIANT == Some("mirror") {
        p.path_scale = -1.2;
        p.lookahead = 0.0;
        p.ke_rudd = 0.0;
        p.ang_accel = 0.0;
        p.yaw_accel = 500.0;
        p.err_corr_p = 1.5;
    }
    let schedule = schedule::Schedule::load(c, "trick72.txt")?.ok_or(Error::Invalid)?;
    let first = input(c, 0)?;
    let eval = path::Evaluation {
        speed: controller::target_speed(c, first, p)?,
        entry_rate: p.entry_rate,
        announce: true,
    };
    let mut path = schedule.build(c, 400.0, eval)?;
    let count = if VARIANT == Some("mirror") { 200 } else { 0 };
    let mut controller = Controller::start(c, &mut path, first, p, 0.0, count)?;
    for i in 1..=512 {
        if VARIANT.is_some() {
            // Deliberately force coverage of every maneuver, independent of
            // these synthetic observations; the flight test checks dynamics.
            controller.path_t = i as f32 / 520.0;
        }
        let step = controller.step(c, &mut path, input(c, i)?, p)?;
        let Step::Command(out) = step else {
            return Err(Error::Invalid);
        };
        oracle.record(
            c,
            "ARCT",
            "Id,Thr,R,P,Y,Rudd,Perr,Pt,Dt,Ert,Qct",
            &[
                L::U16(i),
                L::Float(out.throttle),
                L::Float(out.rates.x),
                L::Float(out.rates.y),
                L::Float(out.rates.z),
                L::Float(out.rudder),
                L::Float(out.position_error),
                L::Float(out.path_t),
                L::Float(out.dt),
                L::Float(out.path_error_t),
                L::Float(out.q_change_t),
            ],
        )?;
        oracle.record(
            c,
            "ARCS",
            "Id,X,Y,Z,Q1,Q2,Q3,Q4,Cx,Cy,Cz,Px,Py,Pz,Ex",
            &[
                L::U16(i),
                L::Float(out.track_pos.x),
                L::Float(out.track_pos.y),
                L::Float(out.track_pos.z),
                L::Float(out.track_attitude.q1),
                L::Float(out.track_attitude.q2),
                L::Float(out.track_attitude.q3),
                L::Float(out.track_attitude.q4),
                L::Float(out.correction_rates.x),
                L::Float(out.correction_rates.y),
                L::Float(out.correction_rates.z),
                L::Float(out.path_rates.x),
                L::Float(out.path_rates.y),
                L::Float(out.path_rates.z),
                L::Float(out.roll_error_rate),
            ],
        )?;
        oracle.record(
            c,
            "RASP",
            "Id,Targ,Curr,P,I,Total,Add",
            &[
                L::U16(i),
                L::Float(out.speed_log[0]),
                L::Float(out.speed_log[1]),
                L::Float(out.speed_log[2]),
                L::Float(out.speed_log[3]),
                L::Float(out.speed_log[4]),
                L::Float(out.speed_log[5]),
            ],
        )?;
        // Round-trip the portable snapshot, rebuilding all static path data periodically.
        let mut words = [0u32; 512];
        controller.save(&mut words[24..60]);
        let mut cursor = 64;
        path.save_cache(&mut words, &mut cursor)?;
        let hash = words[..cursor]
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .fold(2166136261u32, |h, b| (h ^ b as u32).wrapping_mul(16777619));
        c.logger().write(
            "ARHS",
            "Id,Lo,Hi,N",
            None,
            &[
                L::Float(i as f32),
                L::Float((hash & 65535) as f32),
                L::Float((hash >> 16) as f32),
                L::Float(cursor as f32),
            ],
        )?;
        controller = Controller::restore(&words[24..60])?;
        if i % 17 == 0 {
            drop(path);
            path = schedule.build(
                c,
                400.0,
                path::Evaluation {
                    announce: false,
                    ..eval
                },
            )?;
        }
        let mut restored = 64;
        path.restore_cache(&words[..cursor], &mut restored)?;
        if restored != cursor {
            return Err(Error::Invalid);
        }
        if !cfg!(target_os = "none") {
            c.sleep_ms(5)?;
        }
    }
    oracle.finish(c)?;
    let last = input(c, 512)?;
    if !matches!(controller.step(c, &mut path, last, p)?, Step::Waiting) {
        return Err(Error::Invalid);
    }
    let mut bad = last;
    bad.velocity.x = f32::NAN;
    if !matches!(controller.step(c, &mut path, bad, p), Err(Error::Invalid)) {
        return Err(Error::Invalid);
    }
    bad = last;
    bad.attitude = Quaternion::new(0.0, 0.0, 0.0, 0.0);
    if !matches!(controller.step(c, &mut path, bad, p), Err(Error::Invalid)) {
        return Err(Error::Invalid);
    }
    let mut bad_p = p;
    bad_p.err_corr_p = f32::NAN;
    if !matches!(
        controller.step(c, &mut path, last, bad_p),
        Err(Error::Invalid)
    ) {
        return Err(Error::Invalid);
    }
    bad_p = p;
    bad_p.airspeed_min = 0.0;
    if !matches!(
        controller.step(c, &mut path, last, bad_p),
        Err(Error::Invalid)
    ) {
        return Err(Error::Invalid);
    }
    bad = input(c, 513)?;
    bad.pos.z = 100.0;
    if !matches!(
        controller.step(c, &mut path, bad, p)?,
        Step::AltitudeAbort(_)
    ) {
        return Err(Error::Invalid);
    }
    controller.path_t = 1.0;
    if !matches!(
        controller.step(c, &mut path, input(c, 514)?, p)?,
        Step::Finished
    ) {
        return Err(Error::Invalid);
    }
    let old_count = controller.count;
    controller = Controller::start(c, &mut path, first, p, 0.0, old_count)?;
    if controller.count != old_count + 1 {
        return Err(Error::Invalid);
    }
    let mut backwards = first;
    backwards.now -= 1.0;
    if !matches!(
        controller.step(c, &mut path, backwards, p),
        Err(Error::Invalid)
    ) {
        return Err(Error::Invalid);
    }
    // Large position quantization erases the tiny initial tangent, but not
    // the full step. Tiny lookahead then samples the same point as that step.
    // Both degeneracies must use the Lua-compatible fallbacks, not panic or
    // trigger AP_Math's zero-quaternion internal error.
    let mut line = path::Path::leaf(
        c,
        primitives::Geometry::Straight(100.0),
        primitives::Roll::Angle(0.0),
    )?;
    let mut coarse = first;
    coarse.pos = Vector3f::new(2000000.0, 0.0, -30.0);
    coarse.velocity = Vector3f::new(18.0, 0.0, 0.0);
    coarse.eas2tas = 1.0;
    coarse.airspeed = 18.0;
    coarse.attitude = Quaternion::default();
    let mut fallback_p = p;
    fallback_p.path_scale = 1.0;
    fallback_p.lookahead = f32::EPSILON;
    fallback_p.time_corr_p = 0.0;
    let mut fallback = Controller::start(c, &mut line, coarse, fallback_p, 0.0, 200)?;
    coarse.now += 0.025;
    let Step::Command(output) = fallback.step(c, &mut line, coarse, fallback_p)? else {
        return Err(Error::Invalid);
    };
    if !output.path_rate_fallback
        || !output.lookahead_fallback
        || output.path_rates != Vector3f::default()
    {
        return Err(Error::Invalid);
    }
    c.text(
        6,
        if cfg!(target_os = "none") {
            "MATH ok"
        } else {
            "AEROP controller done"
        },
    )
}

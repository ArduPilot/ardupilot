// SPDX-License-Identifier: GPL-3.0-or-later
//! TIME_AXIS_CONFIG packets and collective RTH scenes, as the fork's
//! AC_DroneShowManager_Commands.cpp time axis handler.

use crate::loader::{Show, SCENE_TAG_CRTH, SCENE_TAG_MAIN_SHOW};
use crate::mavlink::{u16_at, u32_at, u64_at};
use crate::show::{
    controller::{Scene, Screenplay, SCENE_FLAG_DYNAMIC_DURATION},
    fmath::hypotf,
    lights::{LightProgram, Rgb},
    rth_plan::{self, RthPlan, ENTRY_HAS_TARGET_XY},
    time_axis::TimeSegment,
    trajectory::Player,
    Vec3,
};
use alloc::vec::Vec;

const HEADER_LEN: usize = 12;
const SCENE_LEN: usize = 7;
const ENTRY_LEN: usize = 8;
const CRTH_SCENE: u16 = 0xC000;
pub const RTH_COLOR: Rgb = Rgb::new(255, 128, 0);
const START_END_XY_THRESHOLD_MM: f32 = 500.0;

pub struct Header {
    pub seq: u8,
    pub num_scenes: u8,
    /// UNIX ms, 0 when unset
    pub start_time_msec: u64,
}

/// Header of a TIME_AXIS_CONFIG body, if the body can hold all scene headers.
pub fn header(d: &[u8]) -> Option<Header> {
    if d.len() < HEADER_LEN {
        return None;
    }
    let num_scenes = d[1];
    if d.len() < HEADER_LEN + num_scenes as usize * SCENE_LEN {
        return None;
    }
    Some(Header { seq: d[0], num_scenes, start_time_msec: u64_at(d, 4) })
}

/// Records `seq`; false for duplicates and for packets out of order
/// (redundant links), which are ignored.
pub fn accept_seq(last: &mut u16, seq: u8) -> bool {
    if seq as u16 == *last {
        return false;
    }
    if *last <= 0xFF && seq.wrapping_sub(*last as u8) >= 0xF0 {
        return false;
    }
    *last = seq as u16;
    true
}

/// Start time moves of 100 ms or more are refused outside the stages where
/// the start time may change (GPS mode only).
pub fn start_time_move_allowed(h: &Header, current_start_unix_usec: u64, change_allowed: bool) -> bool {
    let diff = h.start_time_msec as i64 - (current_start_unix_usec / 1000) as i64;
    change_allowed || (diff < 100 && diff > -100)
}

pub struct CrthTrigger {
    pub rth_start_sec: f32,
    /// show frame, mm
    pub start: Vec3,
}

/// Extends a dynamic scene to cover the rest of its trajectory (up to the
/// landing time for the main show).
fn cover_trajectory(show: &Show, scene: &mut Scene, initial_rate: f32, final_rate: f32) -> Option<()> {
    let mut d = scene.uncovered_trajectory_duration_sec().ok()?;
    if scene.tag == SCENE_TAG_MAIN_SHOW {
        let s = &show.stats;
        if s.duration_sec.is_finite()
            && s.landing_time_sec.is_finite()
            && s.duration_sec >= 0.0
            && s.landing_time_sec >= 0.0
            && s.landing_time_sec <= s.duration_sec
        {
            d -= s.duration_sec - s.landing_time_sec;
        }
    }
    if d <= 0.0 {
        return Some(());
    }
    scene.time_axis.push(TimeSegment::warped(d, initial_rate, final_rate)?).ok()
}

fn crth_scene(show: &Show, plan: &RthPlan, rth_start: f32, wait_or_takeoff: bool) -> Option<(Scene, CrthTrigger)> {
    let mut entry = plan.evaluate_at(rth_start).ok()?;
    let start = if let Some(i) = show.controller.current_scene() {
        // as the fork, evaluate the active scene's trajectory, which is the
        // previous CRTH trajectory when a CRTH is resent
        let traj = show.controller.screenplay().scenes.get(i)?.trajectory.as_ref()?;
        Player::new(traj).position_at(traj, rth_start)
    } else if wait_or_takeoff {
        entry = RthPlan::default_entry(Some(plan), 0.0);
        show.takeoff_position_mm
    } else {
        return None;
    };
    // the CRTH scene's clock starts at 0
    entry.time_sec = 0.0;
    if show.circular && show.modified_for_landing && entry.flags & ENTRY_HAS_TARGET_XY != 0 {
        let p = show.stats.initial_pos;
        if hypotf(entry.landing_target.0 - p.x, entry.landing_target.1 - p.y) < START_END_XY_THRESHOLD_MM {
            let l = show.stats.pos_at_landing_time;
            entry.landing_target = (l.x, l.y);
        }
    }
    let scene = Scene {
        trajectory: Some(rth_plan::trajectory_from_entry(&entry, start).ok()?),
        lights: Some(LightProgram::constant_color(RTH_COLOR)),
        tag: SCENE_TAG_CRTH,
        ..Default::default()
    };
    Some((scene, CrthTrigger { rth_start_sec: rth_start, start }))
}

/// Builds the screenplay described by a TIME_AXIS_CONFIG body; None if any
/// part is invalid, so the current screenplay stays in place.
pub fn build(show: &Show, d: &[u8], h: &Header, wait_or_takeoff: bool) -> Option<(Screenplay, Vec<CrthTrigger>)> {
    let rth = show.controller.screenplay().rth_plan.clone();
    let mut sp = Screenplay { scenes: Vec::new(), rth_plan: rth };
    let mut triggers = Vec::new();
    let mut o = HEADER_LEN;
    for _ in 0..h.num_scenes {
        let sh = d.get(o..o + SCENE_LEN)?;
        o += SCENE_LEN;
        let (num_entries, origin, id) = (sh[0], u32_at(sh, 1), u16_at(sh, 5));
        let mut scene = if id == 0 {
            let m = &show.main_scene;
            Scene {
                trajectory: m.trajectory.clone(),
                lights: m.lights.clone(),
                yaw: m.yaw.clone(),
                events: m.events.clone(),
                tag: SCENE_TAG_MAIN_SHOW,
                ..Default::default()
            }
        } else if id & CRTH_SCENE == CRTH_SCENE {
            let (scene, t) = crth_scene(show, sp.rth_plan.as_ref()?, (id & 0x3FFF) as f32, wait_or_takeoff)?;
            triggers.push(t);
            scene
        } else {
            return None;
        };
        // the fork ignores an invalid (negative) origin
        let _ = scene.time_axis.set_origin_msec(origin as i32);
        for _ in 0..num_entries {
            let e = d.get(o..o + ENTRY_LEN)?;
            o += ENTRY_LEN;
            let ir = u16_at(e, 0) as f32 / 65535.0;
            let fr = u16_at(e, 2) as f32 / 65535.0;
            let duration = u32_at(e, 4);
            if duration == 0 {
                scene.flags |= SCENE_FLAG_DYNAMIC_DURATION;
                cover_trajectory(show, &mut scene, ir, fr)?;
            } else {
                scene.time_axis.push(TimeSegment::new(duration, ir, fr)).ok()?;
            }
        }
        scene.duration_msec = scene.time_axis.total_duration_msec();
        sp.scenes.push(scene);
    }
    Some((sp, triggers))
}

/// SBSP sequence number field.
pub fn log_seq(show: &Show) -> u8 {
    if show.last_seq <= 0xFF {
        show.last_seq as u8
    } else {
        0xFF
    }
}

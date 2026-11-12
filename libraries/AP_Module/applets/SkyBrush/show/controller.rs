// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush control.c and screenplay/{screenplay,scene}.c (Copyright CollMot Robotics Ltd).
//! Screenplay (list of scenes, each with its own time axis) and the show
//! controller that maps wall clock ms to position, velocity, colour, yaw and
//! due events.

use alloc::vec::Vec;

use super::binary::ShowFile;
use super::events::{Event, EventList, EventPlayer};
use super::lights::{LightPlayer, LightProgram, Rgb};
use super::rth_plan::RthPlan;
use super::time_axis::TimeAxis;
use super::trajectory::{Player, Trajectory};
use super::yaw::{YawControl, YawPlayer};
use super::{Error, Result, Vec3};

/// The scene duration may be changed when its trajectory changes.
pub const SCENE_FLAG_DYNAMIC_DURATION: u8 = 1;

#[derive(Clone, Debug, PartialEq)]
pub struct Scene {
    pub trajectory: Option<Trajectory>,
    pub lights: Option<LightProgram>,
    pub yaw: Option<YawControl>,
    pub events: Option<EventList>,
    pub time_axis: TimeAxis,
    /// Wall clock duration from the time axis origin; u32::MAX is infinite.
    pub duration_msec: u32,
    pub tag: u8,
    pub flags: u8,
}

impl Default for Scene {
    fn default() -> Self {
        Scene {
            trajectory: None,
            lights: None,
            yaw: None,
            events: None,
            time_axis: TimeAxis::default(),
            duration_msec: u32::MAX,
            tag: 0,
            flags: 0,
        }
    }
}

fn optional<T>(r: Result<T>) -> Result<Option<T>> {
    match r {
        Ok(v) => Ok(Some(v)),
        Err(Error::NoEnt) => Ok(None),
        Err(e) => Err(e),
    }
}

impl Scene {
    /// Trajectory is required; light program, yaw control and events are optional.
    pub fn from_show(file: &ShowFile) -> Result<Self> {
        Ok(Scene {
            trajectory: Some(Trajectory::from_show(file)?),
            lights: optional(LightProgram::from_show(file))?,
            yaw: optional(YawControl::from_show(file))?,
            events: optional(EventList::from_show(file))?,
            ..Default::default()
        })
    }

    pub fn origin_msec(&self) -> i32 {
        self.time_axis.origin_msec()
    }

    pub fn contains_time_msec(&self, t: u32) -> bool {
        let origin = self.origin_msec();
        if origin >= 0 && t < origin as u32 {
            return false;
        }
        self.duration_msec == u32::MAX || self.duration_msec > t.wrapping_sub(origin as u32)
    }

    pub fn has_flag(&self, flag: u8) -> bool {
        self.flags & flag != 0
    }

    /// Trajectory duration (s) not yet covered by the time axis segments.
    pub fn uncovered_trajectory_duration_sec(&self) -> Result<f32> {
        let warped = self.time_axis.total_warped_duration_sec();
        if !warped.is_finite() {
            if warped.is_nan() || warped < 0.0 {
                return Err(Error::Inval);
            }
            return Ok(0.0);
        }
        Ok(self.trajectory.as_ref().map_or(0.0, |t| t.total_duration_sec() - warped))
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Screenplay {
    pub scenes: Vec<Scene>,
    pub rth_plan: Option<RthPlan>,
}

impl Screenplay {
    /// One scene and the optional RTH plan from a .skyb file.
    pub fn from_show_bytes(data: &[u8]) -> Result<Self> {
        if data.is_empty() {
            return Ok(Screenplay::default());
        }
        let file = ShowFile::parse(data)?;
        Ok(Screenplay { scenes: alloc::vec![Scene::from_show(&file)?], rth_plan: optional(RthPlan::from_show(&file))? })
    }

    /// Show clock ms at which the first scene with `tag` reaches warped time
    /// `warped` (s); u32::MAX if never, or if an earlier scene is active then.
    pub fn time_msec_for_tag_and_warped_time(&self, tag: u8, warped: f32) -> u32 {
        let Some(idx) = self.find_scene_with_tag(tag) else {
            return u32::MAX;
        };
        if warped.is_nan() {
            return u32::MAX;
        }
        let scene = &self.scenes[idx];
        let origin = scene.origin_msec();
        let t = scene.time_axis.reverse_map(warped);
        if t < origin || t == i32::MAX {
            return u32::MAX;
        }
        if scene.duration_msec != u32::MAX && (t.wrapping_sub(origin) as u32) > scene.duration_msec {
            return u32::MAX;
        }
        if self.scenes[..idx].iter().any(|s| s.contains_time_msec(t as u32)) {
            return u32::MAX;
        }
        t as u32
    }

    pub fn time_sec_for_tag_and_warped_time(&self, tag: u8, warped: f32) -> f32 {
        match self.time_msec_for_tag_and_warped_time(tag, warped) {
            u32::MAX => f32::INFINITY,
            t => t as f32 / 1000.0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.scenes.is_empty()
    }

    pub fn scene_at_time_msec(&self, t: u32) -> Option<usize> {
        self.scenes.iter().position(|s| s.contains_time_msec(t)).filter(|&i| i < u16::MAX as usize)
    }

    pub fn find_scene_with_tag(&self, tag: u8) -> Option<usize> {
        self.scenes.iter().position(|s| s.tag == tag)
    }
}

/// Controller output; `None` components are not set.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Output {
    /// mm
    pub position: Option<Vec3>,
    /// mm/s, scaled by the time axis rate
    pub velocity: Option<Vec3>,
    pub color: Option<Rgb>,
    /// degrees
    pub yaw: Option<f32>,
    /// deg/s
    pub yaw_rate: Option<f32>,
}

const DEFAULT_OUTPUT: Output =
    Output { position: None, velocity: Some(Vec3::ZERO), color: None, yaw: None, yaw_rate: Some(0.0) };

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OutputTime {
    /// u32::MAX when the output is invalid.
    pub time_msec: u32,
    pub scene: Option<usize>,
    pub time_in_scene_msec: u32,
    pub warped_time_in_scene_sec: f32,
}

const INVALID_TIME: OutputTime =
    OutputTime { time_msec: u32::MAX, scene: None, time_in_scene_msec: u32::MAX, warped_time_in_scene_sec: 0.0 };

#[derive(Clone, Debug)]
pub struct Controller {
    screenplay: Screenplay,
    current_scene: Option<usize>,
    traj_player: Option<Player>,
    light_player: Option<LightPlayer>,
    yaw_player: Option<YawPlayer>,
    event_player: Option<EventPlayer>,
    output: Output,
    output_time: OutputTime,
    reached_end: bool,
}

impl Controller {
    pub fn new(screenplay: Screenplay) -> Self {
        Controller {
            screenplay,
            current_scene: None,
            traj_player: None,
            light_player: None,
            yaw_player: None,
            event_player: None,
            output: DEFAULT_OUTPUT,
            output_time: INVALID_TIME,
            reached_end: false,
        }
    }

    pub fn from_show_bytes(data: &[u8]) -> Result<Self> {
        Ok(Self::new(Screenplay::from_show_bytes(data)?))
    }

    pub fn screenplay(&self) -> &Screenplay {
        &self.screenplay
    }

    /// Call `notify_screenplay_changed` (or `reset_players`) after changes.
    pub fn screenplay_mut(&mut self) -> &mut Screenplay {
        &mut self.screenplay
    }

    pub fn set_screenplay(&mut self, screenplay: Screenplay) {
        self.screenplay = screenplay;
        self.reset_players();
    }

    /// Invalidates the cached output. Players of the current scene are kept
    /// (so its events do not fire again) unless the scene no longer exists.
    pub fn notify_screenplay_changed(&mut self) {
        if self.current_scene.is_some_and(|i| i >= self.screenplay.scenes.len()) {
            self.current_scene = None;
        }
        self.invalidate_output();
    }

    /// Swaps in a new screenplay. Players of the current scene are kept for
    /// the components that the scene at the same index still plays, so its
    /// events do not fire again.
    pub fn replace_screenplay(&mut self, new: Screenplay) {
        if let Some(i) = self.current_scene {
            match (self.screenplay.scenes.get(i), new.scenes.get(i)) {
                (Some(old), Some(n)) => {
                    if old.trajectory != n.trajectory {
                        self.traj_player = None;
                    }
                    if old.lights != n.lights {
                        self.light_player = None;
                    }
                    if old.yaw != n.yaw {
                        self.yaw_player = None;
                    }
                    if old.events != n.events {
                        self.event_player = None;
                    }
                }
                _ => {
                    self.destroy_players();
                    self.current_scene = None;
                }
            }
        }
        self.screenplay = new;
        self.invalidate_output();
    }

    /// Restarts the trajectory player, e.g. after landing replacement.
    pub fn notify_trajectory_changed(&mut self) {
        self.traj_player = None;
        self.invalidate_output();
        if let Some(i) = self.current_scene {
            self.sync_players(i);
        }
    }

    /// Drops all players so the next update starts them from scratch.
    pub fn reset_players(&mut self) {
        self.destroy_players();
        self.current_scene = None;
        self.invalidate_output();
    }

    pub fn current_scene(&self) -> Option<usize> {
        self.current_scene
    }

    pub fn output(&self) -> &Output {
        &self.output
    }

    pub fn output_time(&self) -> OutputTime {
        self.output_time
    }

    pub fn is_output_valid(&self) -> bool {
        self.output_time.time_msec != u32::MAX
    }

    /// True when the last update fell outside every scene.
    pub fn has_reached_end(&self) -> bool {
        self.reached_end
    }

    /// Trajectory player of the current scene, for extra queries.
    pub fn trajectory_player(&mut self) -> Option<(&mut Player, &Trajectory)> {
        let traj = self.screenplay.scenes.get(self.current_scene?)?.trajectory.as_ref()?;
        Some((self.traj_player.as_mut()?, traj))
    }

    fn invalidate_output(&mut self) {
        self.output = DEFAULT_OUTPUT;
        self.output_time = INVALID_TIME;
        self.reached_end = false;
    }

    fn destroy_players(&mut self) {
        self.traj_player = None;
        self.light_player = None;
        self.yaw_player = None;
        self.event_player = None;
    }

    fn sync_players(&mut self, idx: usize) {
        let Some(scene) = self.screenplay.scenes.get(idx) else {
            self.destroy_players();
            self.current_scene = None;
            return;
        };
        match &scene.trajectory {
            Some(t) => {
                self.traj_player.get_or_insert_with(|| Player::new(t));
            }
            None => self.traj_player = None,
        }
        match &scene.lights {
            Some(l) => {
                self.light_player.get_or_insert_with(|| LightPlayer::new(l));
            }
            None => self.light_player = None,
        }
        match &scene.yaw {
            Some(y) => {
                self.yaw_player.get_or_insert_with(|| YawPlayer::new(y));
            }
            None => self.yaw_player = None,
        }
        if scene.events.is_some() {
            self.event_player.get_or_insert_with(EventPlayer::default);
        } else {
            self.event_player = None;
        }
    }

    fn set_current_scene(&mut self, idx: Option<usize>) {
        if idx != self.current_scene {
            self.destroy_players();
        }
        self.current_scene = idx;
        if let Some(i) = idx {
            self.sync_players(i);
        }
    }

    /// Evaluates the show at wall clock time `t_msec` from the show start.
    pub fn update_time_msec(&mut self, t_msec: u32) -> Result<()> {
        if self.is_output_valid() && t_msec == self.output_time.time_msec {
            return Ok(());
        }
        let idx = if t_msec > i32::MAX as u32 { None } else { self.screenplay.scene_at_time_msec(t_msec) };
        self.set_current_scene(idx);
        self.invalidate_output();

        let (time_in_scene, warped) = match idx {
            None => {
                self.reached_end = true;
                (0, 0.0)
            }
            Some(i) => {
                let scene = &self.screenplay.scenes[i];
                let time_in_scene = t_msec.wrapping_sub(scene.origin_msec() as u32);
                let (warped, rate) = scene.time_axis.map(t_msec as i32);
                let mut out = Output::default();
                if let (Some(p), Some(t)) = (self.traj_player.as_mut(), scene.trajectory.as_ref()) {
                    out.position = Some(p.position_at(t, warped));
                    out.velocity = Some(p.velocity_at(t, warped).scaled(rate));
                }
                if let (Some(p), Some(l)) = (self.light_player.as_mut(), scene.lights.as_ref()) {
                    let w = if warped < 0.0 || !warped.is_finite() {
                        0.0
                    } else if warped > 86400.0 {
                        86400.0
                    } else {
                        warped
                    };
                    out.color = Some(p.color_at(l, (w * 1000.0) as u32 as u64));
                }
                if let (Some(p), Some(y)) = (self.yaw_player.as_mut(), scene.yaw.as_ref()) {
                    out.yaw = Some(p.yaw_at(y, warped));
                    out.yaw_rate = Some(p.yaw_rate_at(y, warped) * rate);
                }
                self.output = out;
                (time_in_scene, warped)
            }
        };
        self.output_time = OutputTime {
            time_msec: t_msec,
            scene: idx,
            time_in_scene_msec: time_in_scene,
            warped_time_in_scene_sec: warped,
        };
        Ok(())
    }

    /// Next event of the current scene due at the current output time.
    pub fn next_event(&mut self) -> Option<Event> {
        let events = self.screenplay.scenes.get(self.current_scene?)?.events.as_ref()?;
        self.event_player.as_mut()?.next_event_not_later_than(events, self.output_time.warped_time_in_scene_sec).copied()
    }
}

/// Wall clock seconds to the controller's ms (negative clamps to 0).
pub fn secs_to_msec(t: f32) -> u32 {
    (t * 1000.0) as u32
}

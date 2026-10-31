// SPDX-License-Identifier: GPL-3.0-or-later
//! Native port of AP_Scripting/applets/quadplane_terrain_avoid.lua.
#![no_std]
#![no_main]

mod parameters;

use ap_rust_sdk::{
    enums::{LocationAltFrame, MavResult, MavSeverity, RcChannelAuxFunc, Rotation},
    CommandIntParams, Context, Error, Location, Parameter, RcChannel, Result,
};
use core::fmt::{self, Write};
use parameters::{read, retry, Parameters};

ap_rust_sdk::module!(run);

const SCRIPT_VERSION: &str = "4.6.0-023";
const STARTUP_DELAY_MS: u32 = 25_000;
const VEHICLE_TYPE_PLANE: u8 = 3;

const MODE_AUTO: u8 = 10;
const MODE_RTL: u8 = 11;
const MODE_LOITER: u8 = 12;
const MODE_GUIDED: u8 = 15;
const MODE_QHOVER: u8 = 18;
const MODE_QLOITER: u8 = 19;
const MODE_QLAND: u8 = 20;
const MODE_QRTL: u8 = 21;

const CMD_DO_CHANGE_SPEED: u16 = 178;
const CMD_DO_REPOSITION: u16 = 192;
const CMD_GUIDED_CHANGE_SPEED: u16 = 43_000;
const CMD_GUIDED_CHANGE_ALTITUDE: u16 = 43_001;
const MAV_FRAME_GLOBAL: u8 = 0;
const AIRSPEED: f32 = 0.0;

const ALT_ABSOLUTE: LocationAltFrame = LocationAltFrame(0);
const RANGE_DOWN: Rotation = Rotation(25);
const RANGE_FORWARD: Rotation = Rotation(0);
const RANGE_GOOD: u8 = 4;
const MAX_RANGE: f32 = 90.0;
const MIN_ALT_MAX: f32 = 20.0;
const PITCH_TOLERANCE: f32 = 1.1;
const QUAD_TOLERANCE: f32 = 1.5;
const RAD_TO_DEG: f32 = 57.295_78;
const PI: f32 = core::f32::consts::PI;

struct Text<const N: usize> {
    bytes: [u8; N],
    length: usize,
}

impl<const N: usize> Text<N> {
    fn new(args: fmt::Arguments<'_>) -> Result<Self> {
        let mut text = Self { bytes: [0; N], length: 0 };
        text.write_fmt(args).map_err(|_| Error::Invalid)?;
        Ok(text)
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.length]).unwrap()
    }
}

impl<const N: usize> Write for Text<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.length + value.len();
        if end > N {
            return Err(fmt::Error);
        }
        self.bytes[self.length..end].copy_from_slice(value.as_bytes());
        self.length = end;
        Ok(())
    }
}

fn message(context: &Context<'_>, severity: i32, args: fmt::Arguments<'_>) -> Result {
    let text = Text::<160>::new(args)?;
    retry(context, || context.gcs().send_text(MavSeverity(severity), text.as_str()))
}

fn bind<'a, 'host>(context: &'a Context<'host>, name: &str) -> Result<Parameter<'a, 'host>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Invalid)
}

fn millis(context: &Context<'_>) -> u64 {
    context.time_us() / 1_000
}

fn same_location(a: Option<Location>, b: Option<Location>) -> bool {
    match (a, b) {
        (Some(a), Some(b)) => {
            a.lat() == b.lat()
                && a.lng() == b.lng()
                && a.alt() == b.alt()
                && a.relative_alt() == b.relative_alt()
                && a.terrain_alt() == b.terrain_alt()
                && a.origin_alt() == b.origin_alt()
        }
        _ => false,
    }
}

fn same_lat_lng(a: Option<Location>, b: Option<Location>) -> bool {
    matches!((a, b), (Some(a), Some(b)) if a.lat() == b.lat() && a.lng() == b.lng())
}

#[derive(Default)]
struct LocationTracker {
    previous: Option<Location>,
    previous_previous: Option<Location>,
}

impl LocationTracker {
    fn update(&mut self, target: Option<Location>) {
        let Some(target) = target else { return };
        if !same_location(self.previous, Some(target)) {
            if self.previous.is_some() && !same_location(self.previous, self.previous_previous) {
                self.previous_previous = self.previous;
            }
            self.previous = Some(target);
        }
        if self.previous_previous.is_none() {
            self.previous_previous = self.previous;
        }
    }
}

struct Applet<'ctx, 'host> {
    p: Parameters<'ctx, 'host>,
    throttle: RcChannel,
    throttle_up_pwm: u16,
    throttle_hover_pwm: u16,

    pitch_down_min: f32,
    pitch_forward_min: f32,
    pitch_timeout: f32,
    home_distance_max: f32,
    quad_down_min: f32,
    quad_forward_min: f32,
    altitude_max: f32,
    groundspeed_max: f32,
    groundspeed_airbrake_limit: f32,
    cmtc_height: f32,
    cmtc_enable: bool,
    cmtc_radius: f32,
    terrain_spacing: f32,
    pitch_limit_max_deg: f32,
    tecs_climb_max: f32,
    airspeed_min: f32,
    airspeed_cruise: f32,
    airspeed_max: f32,
    wp_loiter_radius: f32,
    refresh_ms: u32,

    now_ms: u64,
    parameters_refreshed_ms: u64,
    vehicle_mode: u8,
    current_location: Option<Location>,
    current_target: Option<Location>,
    current_altitude: f32,
    current_wp_bearing_deg: f32,
    current_heading_deg: f32,
    groundspeed: f32,
    airspeed_current: f32,
    airspeed_desired: f32,
    terrain_altitude: f32,
    terrain_max_exceeded: bool,
    range_down: f32,
    range_forward: f32,

    tracker: LocationTracker,
    switch_on: bool,
    last_switch_state: i16,
    close_to_home: bool,
    avoid_enter_mode: i16,
    q_wvane_saved: f32,

    cmtc_active: bool,
    cmtc_target_alt_amsl: f32,
    pre_cmtc_heading_deg: f32,
    pitching_active: bool,
    pre_pitch_mode: u8,
    pre_pitch_altitude: f32,
    pre_pitch_target: Option<Location>,
    pitch_last_good_ms: u64,
    pitch_last_bad_ms: u64,
    pitch_bad_timer: i32,
    quading_active: bool,
    last_quading_location: Option<Location>,
    quad_tolerance: f32,

    slowdown_quading: bool,
    airbrake_trigger: bool,
    airbrake_on: bool,
    airbrake_triggered_ms: u64,
    previous_commanded_speed: f32,
}

impl<'ctx, 'host> Applet<'ctx, 'host> {
    fn new(context: &'ctx Context<'host>, p: Parameters<'ctx, 'host>) -> Result<Self> {
        let throttle_number = read(&bind(context, "RCMAP_THROTTLE")?)? as u8;
        let throttle = retry(context, || context.rc().get_channel(throttle_number))?
            .ok_or(Error::Unavailable)?;
        let prefix = Text::<10>::new(format_args!("RC{throttle_number}_MIN"))?;
        let throttle_min = read(&bind(context, prefix.as_str())?)?;
        let prefix = Text::<10>::new(format_args!("RC{throttle_number}_MAX"))?;
        let throttle_max = read(&bind(context, prefix.as_str())?)?;
        let span = throttle_max - throttle_min;

        let now = millis(context);
        let pitch_down_min = read(&p.pitch_down_min)?;
        let mut cmtc_height = read(&p.cmtc_height)?;
        if cmtc_height == 0.0 {
            cmtc_height = pitch_down_min;
        }
        let wp_loiter_radius = read(&p.wp_loiter_radius)?;
        let mut cmtc_radius = read(&p.cmtc_radius)?;
        if cmtc_radius <= 0.0 {
            cmtc_radius = wp_loiter_radius;
        }
        let update_rate = read(&p.update_rate)?;
        if update_rate <= 0.0 {
            return Err(Error::Invalid);
        }
        let airspeed_current = retry(context, || context.ahrs().airspeed_eas())?.unwrap_or(0.0);
        let airspeed_cruise = read(&p.airspeed_cruise)?;
        let q_wvane_saved = read(&p.q_wvane_enable)?;
        Ok(Self {
            pitch_down_min,
            pitch_forward_min: read(&p.pitch_forward_min)?,
            pitch_timeout: read(&p.pitch_timeout)?,
            home_distance_max: read(&p.home_distance)?,
            quad_down_min: read(&p.quad_down_min)?,
            quad_forward_min: read(&p.quad_forward_min)?,
            altitude_max: read(&p.altitude_max)?,
            groundspeed_max: read(&p.groundspeed_max)?,
            groundspeed_airbrake_limit: read(&p.groundspeed_airbrake)?,
            cmtc_height,
            cmtc_enable: read(&p.cmtc_enable)? == 1.0,
            cmtc_radius,
            terrain_spacing: read(&p.terrain_spacing)?,
            pitch_limit_max_deg: read(&p.pitch_limit_max_deg)?,
            tecs_climb_max: read(&p.tecs_climb_max)?,
            airspeed_min: read(&p.airspeed_min)?,
            airspeed_cruise,
            airspeed_max: read(&p.airspeed_max)?,
            wp_loiter_radius,
            refresh_ms: (1_000.0 / update_rate) as u32,
            p,
            throttle,
            throttle_up_pwm: (span * 0.8 + throttle_min) as u16,
            throttle_hover_pwm: (span * 0.5 + throttle_min) as u16,
            now_ms: now,
            parameters_refreshed_ms: now,
            vehicle_mode: retry(context, || context.vehicle().get_mode())?,
            current_location: retry(context, || context.ahrs().get_location())?,
            current_target: None,
            current_altitude: 0.0,
            current_wp_bearing_deg: 0.0,
            current_heading_deg: -1.0,
            groundspeed: 0.0,
            airspeed_current,
            airspeed_desired: airspeed_current,
            terrain_altitude: retry(context, || context.terrain().height_above_terrain(true))?.unwrap_or(0.0),
            terrain_max_exceeded: false,
            range_down: 0.0,
            range_forward: 0.0,
            tracker: LocationTracker::default(),
            switch_on: true,
            last_switch_state: -1,
            close_to_home: false,
            avoid_enter_mode: -1,
            q_wvane_saved,
            cmtc_active: false,
            cmtc_target_alt_amsl: -1.0,
            pre_cmtc_heading_deg: 0.0,
            pitching_active: false,
            pre_pitch_mode: 0,
            pre_pitch_altitude: 0.0,
            pre_pitch_target: None,
            pitch_last_good_ms: now,
            pitch_last_bad_ms: now,
            pitch_bad_timer: -10,
            quading_active: false,
            last_quading_location: None,
            quad_tolerance: QUAD_TOLERANCE,
            slowdown_quading: false,
            airbrake_trigger: false,
            airbrake_on: false,
            airbrake_triggered_ms: now,
            previous_commanded_speed: -1.0,
        })
    }

    fn set_vehicle_mode(&mut self, context: &Context<'_>, mode: u8) -> Result {
        if mode > 0 && self.vehicle_mode != mode {
            let _ = retry(context, || context.vehicle().set_mode(mode))?;
            self.vehicle_mode = mode;
        }
        Ok(())
    }

    fn set_avoid_mode(&mut self, context: &Context<'_>, mode: u8) -> Result {
        if retry(context, || context.vehicle().get_mode())? != mode {
            self.avoid_enter_mode = self.vehicle_mode as i16;
            self.set_vehicle_mode(context, mode)?;
        }
        Ok(())
    }

    fn reset_avoid_mode(&mut self, context: &Context<'_>) -> Result {
        let mode = match self.avoid_enter_mode as u8 {
            MODE_AUTO | MODE_GUIDED | MODE_LOITER | MODE_QHOVER | MODE_QLOITER | MODE_QRTL
            | MODE_RTL => self.avoid_enter_mode as u8,
            _ => MODE_AUTO,
        };
        self.set_vehicle_mode(context, mode)?;
        self.avoid_enter_mode = -1;
        if mode == MODE_AUTO {
            self.airspeed_desired = self.airspeed_cruise;
            if let Some(mut previous) = self.tracker.previous_previous {
                let _ = retry(context, || context.vehicle().set_crosstrack_start(&mut previous))?;
            }
        }
        Ok(())
    }

    fn disable_wvane(&mut self) -> Result {
        self.q_wvane_saved = read(&self.p.q_wvane_enable)?;
        let _ = self.p.q_wvane_enable.set(0.0)?;
        Ok(())
    }

    fn enable_wvane(&mut self) -> Result {
        let restored = if self.q_wvane_saved >= 0.0 { self.q_wvane_saved } else { 0.0 };
        let _ = self.p.q_wvane_enable.set(restored)?;
        self.q_wvane_saved = -1.0;
        Ok(())
    }

    fn command(context: &Context<'_>, command: u16, params: &CommandIntParams) -> Result<bool> {
        Ok(retry(context, || context.gcs().run_command_int(command, params))? == MavResult(0))
    }

    fn set_speed(&mut self, context: &Context<'_>, speed: f32) -> Result {
        if speed <= 0.0 || speed == self.previous_commanded_speed {
            return Ok(());
        }
        self.previous_commanded_speed = speed;
        let command = if self.vehicle_mode == MODE_GUIDED {
            CMD_GUIDED_CHANGE_SPEED
        } else {
            CMD_DO_CHANGE_SPEED
        };
        let _ = Self::command(
            context,
            command,
            &CommandIntParams {
                p1: AIRSPEED,
                p2: speed,
                frame: MAV_FRAME_GLOBAL,
                ..CommandIntParams::default()
            },
        )?;
        Ok(())
    }

    fn set_altitude_target(&mut self, context: &Context<'_>, altitude: f32) -> Result<f32> {
        self.current_target = retry(context, || context.vehicle().get_target_location())?;
        if let Some(mut current) = self.current_target {
            let mut target = current;
            target.set_alt(context, retry(context, || context.libm().floorf(altitude * 100.0))? as i32)?;
            if !retry(context, || context.vehicle().update_target_location(&mut current, &mut target))? {
                message(context, 3, format_args!("TerrAvoid: failed to set alt: {altitude:.0}"))?;
            }
        }
        Ok(altitude)
    }

    fn set_altitude_high(&mut self, context: &Context<'_>) -> Result<f32> {
        let old = self.current_target.or(self.current_location).map_or(0.0, |location| location.alt() as f32 * 0.01);
        self.set_altitude_target(context, self.current_altitude + 2_000.0)?;
        Ok(old)
    }

    fn refresh_parameters(&mut self) -> Result {
        if self.now_ms.saturating_sub(self.parameters_refreshed_ms) <= 1_000 {
            return Ok(());
        }
        self.pitch_down_min = read(&self.p.pitch_down_min)?;
        self.pitch_forward_min = read(&self.p.pitch_forward_min)?;
        self.pitch_timeout = read(&self.p.pitch_timeout)?;
        self.home_distance_max = read(&self.p.home_distance)?;
        self.quad_down_min = read(&self.p.quad_down_min)?;
        self.quad_forward_min = read(&self.p.quad_forward_min)?;
        self.altitude_max = read(&self.p.altitude_max)?;
        self.groundspeed_max = read(&self.p.groundspeed_max)?;
        self.groundspeed_airbrake_limit = read(&self.p.groundspeed_airbrake)?;
        self.airspeed_min = read(&self.p.airspeed_min)?;
        self.airspeed_cruise = read(&self.p.airspeed_cruise)?;
        self.airspeed_max = read(&self.p.airspeed_max)?;
        self.cmtc_enable = read(&self.p.cmtc_enable)? == 1.0;
        if self.cmtc_enable {
            self.cmtc_height = read(&self.p.cmtc_height)?;
            if self.cmtc_height <= 0.0 {
                self.cmtc_height = self.pitch_down_min;
            }
            self.cmtc_radius = read(&self.p.cmtc_radius)?;
            if self.cmtc_radius <= 0.0 {
                self.cmtc_radius = self.wp_loiter_radius;
            }
        }
        self.parameters_refreshed_ms = self.now_ms;
        Ok(())
    }

    fn check_activation_switch(&mut self, context: &Context<'_>) -> Result {
        let function = read(&self.p.activation_function)? as i32;
        let state = retry(context, || context.rc().get_aux_cached(RcChannelAuxFunc(function)))?
            .map_or(-1, i16::from);
        if state != self.last_switch_state {
            if state == 0 {
                self.switch_on = true;
                message(context, 5, format_args!("TerrAvoid: activated"))?;
                self.pitch_last_good_ms = self.now_ms;
            } else if state == 2 {
                self.switch_on = false;
                message(context, 5, format_args!("TerrAvoid: deactivated"))?;
            }
            self.last_switch_state = state;
        }
        Ok(())
    }

    fn active(&mut self, context: &Context<'_>) -> Result<bool> {
        if !self.switch_on || !retry(context, || context.arming().is_armed())? {
            return Ok(false);
        }
        let allowed = matches!(self.vehicle_mode, MODE_AUTO | MODE_GUIDED | MODE_RTL | MODE_QRTL | MODE_QLAND)
            || ((self.quading_active || self.airbrake_on)
                && matches!(self.vehicle_mode, MODE_QLOITER | MODE_QHOVER));
        if !allowed {
            return Ok(false);
        }
        let mut home = retry(context, || context.ahrs().get_home())?;
        let distance = if let Some(mut current) = self.current_location {
            home.get_distance(context, &mut current)?
        } else {
            0.0
        };
        if distance < self.home_distance_max {
            if !self.close_to_home {
                message(context, 6, format_args!("TerrAvoid: close to home"))?;
                self.close_to_home = true;
            }
            return Ok(false);
        }
        if self.close_to_home {
            message(context, 6, format_args!("TerrAvoid: away from home"))?;
        }
        self.close_to_home = false;
        Ok(true)
    }

    fn populate_rangefinders(&mut self, context: &Context<'_>) -> Result {
        if retry(context, || context.rangefinder().has_data_orient(RANGE_DOWN))?
            && retry(context, || context.rangefinder().status_orient(RANGE_DOWN))? == RANGE_GOOD
        {
            self.range_down = retry(context, || context.rangefinder().distance_orient(RANGE_DOWN))?;
        } else {
            self.range_down = retry(context, || context.terrain().height_above_terrain(true))?.unwrap_or(0.0);
        }
        if retry(context, || context.rangefinder().has_data_orient(RANGE_FORWARD))?
            && retry(context, || context.rangefinder().status_orient(RANGE_FORWARD))? == RANGE_GOOD
        {
            // Preserve the Lua applet's behavior: its forward-sensor branch
            // currently updates the downward sample.
            self.range_down = retry(context, || context.rangefinder().distance_orient(RANGE_FORWARD))?;
        } else {
            self.range_forward = 0.0;
        }
        self.terrain_altitude = retry(context, || context.terrain().height_above_terrain(true))?.unwrap_or(0.0);
        self.terrain_max_exceeded = self.altitude_max > MIN_ALT_MAX && self.terrain_altitude > self.altitude_max;
        if self.range_down <= 0.0 || self.range_down > MAX_RANGE {
            self.range_down = self.terrain_altitude;
        }
        if self.range_forward <= 0.0 {
            self.range_forward = 0.0;
        } else if self.range_forward > MAX_RANGE {
            self.range_forward = MAX_RANGE;
        }
        Ok(())
    }

    fn pitch_obstacle_down(&self, multiplier: f32) -> bool {
        self.range_down > 0.0
            && self.range_down < MAX_RANGE
            && self.range_down < self.pitch_down_min * multiplier
    }

    fn pitch_obstacle_forward(&self, multiplier: f32) -> bool {
        self.range_forward > 0.0
            && self.range_forward < MAX_RANGE
            && self.range_forward < self.pitch_forward_min * multiplier
    }

    fn pitch_obstacle(&self, multiplier: f32) -> bool {
        !self.terrain_max_exceeded
            && (self.pitch_obstacle_down(multiplier) || self.pitch_obstacle_forward(multiplier))
    }

    fn quad_obstacle(&self) -> bool {
        !self.terrain_max_exceeded
            && ((self.range_down > 0.0 && self.range_down < MAX_RANGE && self.range_down < self.quad_down_min)
                || (self.range_forward > 0.0
                    && self.range_forward < MAX_RANGE
                    && self.range_forward < self.quad_forward_min))
    }

    fn pitching_start(&mut self, context: &Context<'_>) -> Result {
        if self.slowdown_quading {
            return self.quading_start(context);
        }
        self.pitching_active = true;
        message(context, 6, format_args!("TerrAvoid: Pitching started"))?;
        self.pre_pitch_mode = self.vehicle_mode;
        self.pre_pitch_altitude = self.set_altitude_high(context)?;
        self.pre_pitch_target = self.current_target;
        self.pitch_last_bad_ms = millis(context);
        Ok(())
    }

    fn pitching_check(&mut self, context: &Context<'_>) -> Result<bool> {
        if self.groundspeed < read(&self.p.pitch_groundspeed_min)? {
            return Ok(false);
        }
        if self.pitch_obstacle(1.0) {
            if !self.pitching_active {
                let since_good = self.now_ms.saturating_sub(self.pitch_last_good_ms) as f32 * 0.001;
                if since_good > self.pitch_timeout {
                    self.pitching_start(context)?;
                    if self.pitch_obstacle_down(1.0) {
                        message(context, 6, format_args!("TerrAvoid: Obstacle down: {:.2} m", self.range_down))?;
                    }
                    if self.pitch_obstacle_forward(1.0) {
                        message(context, 6, format_args!("TerrAvoid: Obstacle forward: {:.2} m", self.range_forward))?;
                    }
                } else if since_good > (self.pitch_bad_timer + 1) as f32 {
                    message(context, 6, format_args!("TerrAvoid: high terrain detected"))?;
                    self.pitch_bad_timer = (since_good + 0.5) as i32;
                }
            }
            return Ok(true);
        }
        if self.pitch_bad_timer >= 0 && !self.terrain_max_exceeded {
            message(context, 6, format_args!("TerrAvoid: terrain Ok"))?;
        }
        self.pitch_last_good_ms = self.now_ms;
        self.pitch_bad_timer = -1;
        Ok(false)
    }

    fn pitching_stop(&mut self, context: &Context<'_>) -> Result {
        if self.pitching_active {
            message(context, 6, format_args!("TerrAvoid: Pitching DONE"))?;
        }
        self.pitching_active = false;
        if self.pre_pitch_mode != self.vehicle_mode {
            return Ok(());
        }
        message(context, 6, format_args!("TerrAvoid: reset Alt: {}", self.pre_pitch_altitude))?;
        self.set_altitude_target(context, self.pre_pitch_altitude)?;
        self.airspeed_desired = self.airspeed_cruise;
        if let Some(mut previous) = self.tracker.previous_previous {
            let _ = retry(context, || context.vehicle().set_crosstrack_start(&mut previous))?;
        }
        Ok(())
    }

    fn pitching_update(&mut self, context: &Context<'_>) -> Result {
        if self.quading_active {
            self.pitching_active = false;
            return Ok(());
        }
        let passed_target = !same_lat_lng(self.pre_pitch_target, self.current_target);
        if (!self.pitch_obstacle(PITCH_TOLERANCE)
            && self.now_ms.saturating_sub(self.pitch_last_bad_ms) as f32 * 0.001 > self.pitch_timeout * 2.0)
            || passed_target
        {
            self.pitching_stop(context)?;
        } else {
            self.set_altitude_high(context)?;
            if self.pitch_obstacle(PITCH_TOLERANCE) {
                self.pitch_last_bad_ms = millis(context);
            }
            self.airspeed_desired = self.airspeed_cruise;
        }
        Ok(())
    }

    fn quading_start(&mut self, context: &Context<'_>) -> Result {
        message(context, 6, format_args!("TerrAvoid: Quading started"))?;
        if self.slowdown_quading {
            self.slowdown_quading = false;
        } else {
            if self.cmtc_active {
                message(
                    context,
                    3,
                    format_args!("TerrAvoid: Quading overrides CMTC: {}", self.cmtc_target_alt_amsl),
                )?;
                self.set_vehicle_mode(context, MODE_QLOITER)?;
            } else {
                self.set_avoid_mode(context, MODE_QLOITER)?;
            }
            self.disable_wvane()?;
        }
        self.quading_active = true;
        self.pitching_active = false;
        self.cmtc_active = false;
        self.throttle.set_override(context, self.throttle_up_pwm)?;
        if let Some(mut current) = self.current_location {
            if let Some(mut last) = self.last_quading_location {
                if current.get_distance(context, &mut last)? < self.wp_loiter_radius {
                    message(context, 6, format_args!("TerrAvoid: Quading repeat: {}", self.quad_tolerance))?;
                    self.quad_tolerance *= 1.5;
                } else {
                    self.quad_tolerance = QUAD_TOLERANCE;
                }
            }
            self.last_quading_location = Some(current);
        }
        Ok(())
    }

    fn quading_check(&mut self, context: &Context<'_>) -> Result<bool> {
        if self.quad_obstacle() && !self.quading_active {
            if self.range_down > 0.0 && self.range_down < MAX_RANGE && self.range_down < self.quad_down_min {
                message(context, 6, format_args!("TerrAvoid: Obstacle down: {:.2} m", self.range_down))?;
            }
            if self.range_forward > 0.0
                && self.range_forward < MAX_RANGE
                && self.range_forward < self.quad_forward_min
            {
                message(context, 6, format_args!("TerrAvoid: Obstacle forward: {:.2} m", self.range_forward))?;
            }
            if self.pitching_active {
                message(context, 6, format_args!("TerrAvoid: Stop Pitching"))?;
                self.pitching_stop(context)?;
            }
            self.quading_start(context)?;
            return Ok(true);
        }
        Ok(false)
    }

    fn quading_stop(&mut self, context: &Context<'_>) -> Result {
        if self.quading_active {
            message(context, 6, format_args!("TerrAvoid: Quading DONE"))?;
        }
        self.quading_active = false;
        self.airbrake_on = false;
        self.enable_wvane()?;
        if self.vehicle_mode != MODE_QLOITER {
            message(context, 5, format_args!("TerrAvoid: exit Quading NOT QLoiter"))?;
            self.avoid_enter_mode = 0;
            return Ok(());
        }
        self.reset_avoid_mode(context)
    }

    fn quading_update(&mut self, context: &Context<'_>) -> Result {
        if !self.pitch_obstacle(self.quad_tolerance) || self.terrain_max_exceeded {
            self.quading_stop(context)
        } else {
            self.throttle.set_override(context, self.throttle_up_pwm)
        }
    }

    fn wrap_360(context: &Context<'_>, angle: f32) -> Result<f32> {
        let value = retry(context, || context.libm().fmodf(angle, 360.0))?;
        Ok(if value < 0.0 { value + 360.0 } else { value })
    }

    fn terrain_lookahead(
        &self,
        context: &Context<'_>,
        mut start: Location,
        bearing: f32,
        mut distance: f32,
        ratio: f32,
    ) -> Result<Option<Location>> {
        let mut highest = None;
        let mut climb = 0.0;
        let mut highest_rise = 0.0;
        let _ = start.change_alt_frame(context, ALT_ABSOLUTE)?;
        let Some(base_height) = retry(context, || context.terrain().height_amsl(&mut start, true))? else {
            return Ok(None);
        };
        while distance > 0.0 {
            start.offset_bearing(context, bearing, self.terrain_spacing)?;
            climb += ratio * self.terrain_spacing;
            if let Some(height) = retry(context, || context.terrain().height_amsl(&mut start, true))? {
                let rise = height - base_height - climb;
                if rise > highest_rise {
                    highest_rise = rise;
                    let mut location = start;
                    location.set_alt(context, retry(context, || context.libm().floorf(height * 100.0))? as i32)?;
                    highest = Some(location);
                }
            }
            distance -= self.terrain_spacing;
        }
        Ok(highest)
    }

    fn terrain_approaching(&self, context: &Context<'_>, clearance: f32) -> Result<(f32, f32)> {
        let (Some(mut current), Some(mut target)) = (self.current_location, self.current_target) else {
            return Ok((0.0, -1.0));
        };
        let distance = current.get_distance(context, &mut target)?;
        let bearing = current.get_bearing(context, &mut target)? * RAD_TO_DEG;
        if self.groundspeed <= 0.0 {
            return Ok((0.0, -1.0));
        }
        let ratio = 0.5 * self.tecs_climb_max / self.groundspeed;
        let Some(mut highest) = self.terrain_lookahead(context, current, bearing, distance, ratio)? else {
            return Ok((0.0, -1.0));
        };
        let _ = current.change_alt_frame(context, ALT_ABSOLUTE)?;
        let _ = highest.change_alt_frame(context, ALT_ABSOLUTE)?;
        let difference = highest.alt() as f32 * 0.01 + clearance - current.alt() as f32 * 0.01;
        if difference <= 0.0 {
            return Ok((0.0, -1.0));
        }
        let highest_distance = current.get_distance(context, &mut highest)?;
        let pitch = retry(context, || context.libm().atan2f(difference, highest_distance))? * RAD_TO_DEG;
        Ok((pitch, highest.alt() as f32 * 0.01 + clearance * 1.5))
    }

    fn arc_terrain_altitude(
        &self,
        context: &Context<'_>,
        center: Location,
        bearing_start: f32,
        step: f32,
        radius: f32,
    ) -> Result<f32> {
        let mut increment = step;
        let mut highest: f32 = 0.0;
        while increment.abs() < 180.0 {
            let bearing = Self::wrap_360(context, bearing_start + increment)?;
            let mut edge = center;
            edge.offset_bearing(context, bearing, radius)?;
            highest = highest.max(retry(context, || context.terrain().height_amsl(&mut edge, true))?.unwrap_or(0.0));
            increment += step;
        }
        Ok(highest)
    }

    fn avoid_terrain(&mut self, context: &Context<'_>, target_altitude: f32) -> Result<&'static str> {
        let Some(current) = self.current_location else { return Ok("unknown") };
        let scaled_radius = self.cmtc_radius * (1.0 + self.current_altitude / 3_000.0);
        let mut left = current;
        let mut right = current;
        left.offset_bearing(
            context,
            Self::wrap_360(context, self.current_wp_bearing_deg - 90.0)?,
            scaled_radius,
        )?;
        right.offset_bearing(
            context,
            Self::wrap_360(context, self.current_wp_bearing_deg + 90.0)?,
            scaled_radius,
        )?;
        let spacing_deg = self.terrain_spacing * 360.0 / (2.0 * PI * scaled_radius);
        let left_height = self.arc_terrain_altitude(
            context,
            left,
            self.current_wp_bearing_deg,
            -spacing_deg,
            scaled_radius,
        )?;
        let right_height = self.arc_terrain_altitude(
            context,
            right,
            self.current_wp_bearing_deg,
            spacing_deg,
            scaled_radius,
        )?;
        self.set_avoid_mode(context, MODE_GUIDED)?;
        let roll = retry(context, || context.ahrs().get_roll_rad())? * RAD_TO_DEG;
        let (direction, center, yaw) = if left_height < right_height || roll > 30.0 {
            ("left", left, 1.0)
        } else {
            ("right", right, 0.0)
        };
        let _ = Self::command(
            context,
            CMD_DO_REPOSITION,
            &CommandIntParams {
                p1: -1.0,
                p3: self.cmtc_radius,
                p4: yaw,
                x: center.lat(),
                y: center.lng(),
                z: target_altitude + 50.0,
                frame: MAV_FRAME_GLOBAL,
                ..CommandIntParams::default()
            },
        )?;
        let _ = Self::command(
            context,
            CMD_GUIDED_CHANGE_ALTITUDE,
            &CommandIntParams {
                p3: 1_000.0,
                z: target_altitude + 50.0,
                frame: MAV_FRAME_GLOBAL,
                ..CommandIntParams::default()
            },
        )?;
        self.airspeed_desired = self.airspeed_cruise;
        Ok(direction)
    }

    fn cmtc_start(&mut self, context: &Context<'_>, target_altitude: f32) -> Result {
        if self.cmtc_active || self.terrain_altitude > self.altitude_max {
            return Ok(());
        }
        self.cmtc_active = true;
        self.pre_cmtc_heading_deg = self.current_heading_deg;
        self.cmtc_target_alt_amsl = target_altitude - 3.0;
        let direction = self.avoid_terrain(context, target_altitude)?;
        message(
            context,
            6,
            format_args!("TerrAvoid: CMTC loiter {direction} to {:.0} alt", self.cmtc_target_alt_amsl),
        )?;
        self.airspeed_desired = self.airspeed_max;
        Ok(())
    }

    fn cmtc_stop(&mut self, context: &Context<'_>) -> Result {
        if self.current_location.is_some() && self.cmtc_active {
            message(context, 6, format_args!("TerrAvoid: CMTC Done alt: {:.0}", self.current_altitude))?;
        }
        self.cmtc_active = false;
        self.cmtc_target_alt_amsl = -1.0;
        self.reset_avoid_mode(context)
    }

    fn cmtc_update(&mut self, context: &Context<'_>) -> Result {
        let Some(mut current) = self.current_location else { return Ok(()) };
        let _ = current.change_alt_frame(context, ALT_ABSOLUTE)?;
        if self.terrain_altitude > self.altitude_max
            || (self.current_altitude > self.cmtc_target_alt_amsl
                && (self.pre_cmtc_heading_deg - self.current_heading_deg).abs() < 45.0)
        {
            message(
                context,
                6,
                format_args!(
                    "TerrAvoid: CMTC STOP alt curr {:.0} trg {:.0} max {:.1}",
                    self.current_altitude,
                    current.alt() as f32 * 0.01,
                    self.altitude_max
                ),
            )?;
            self.cmtc_stop(context)?;
        } else {
            self.airspeed_desired = self.airspeed_cruise;
            self.set_speed(context, self.airspeed_desired)?;
        }
        Ok(())
    }

    fn slowdown_airbrake_end(&mut self, context: &Context<'_>) -> Result {
        self.airbrake_trigger = false;
        if !self.airbrake_on {
            return Ok(());
        }
        self.airbrake_on = false;
        self.enable_wvane()?;
        message(context, 5, format_args!("TerrAvoid: airbraking stopping"))?;
        if self.vehicle_mode == MODE_QLOITER {
            self.reset_avoid_mode(context)?;
        }
        Ok(())
    }

    fn slow_down(&mut self, context: &Context<'_>, error: f32) -> Result<f32> {
        let new_speed = (self.airspeed_current + error).max(self.airspeed_min).min(self.airspeed_max);
        if self.groundspeed_airbrake_limit != 0.0
            && !self.airbrake_on
            && error < self.groundspeed_airbrake_limit
        {
            if !self.airbrake_trigger {
                self.airbrake_trigger = true;
                self.airbrake_triggered_ms = millis(context);
            }
            if self.now_ms.saturating_sub(self.airbrake_triggered_ms) as f32 * 0.001 > self.pitch_timeout {
                self.airbrake_on = true;
                message(
                    context,
                    5,
                    format_args!(
                        "TerrAvoid: airbrake current {} error {} new {}",
                        self.airspeed_current,
                        error,
                        new_speed
                    ),
                )?;
                message(context, 5, format_args!("TerrAvoid: airbraking Starting"))?;
                self.disable_wvane()?;
                self.set_avoid_mode(context, MODE_QLOITER)?;
            }
        } else if self.airbrake_trigger {
            self.airbrake_trigger = false;
        }
        if self.airbrake_on {
            if error >= 0.0 || self.now_ms.saturating_sub(self.airbrake_triggered_ms) > 10_000 {
                self.slowdown_airbrake_end(context)?;
            } else {
                self.throttle.set_override(context, self.throttle_hover_pwm)?;
            }
        }
        Ok(new_speed)
    }

    fn stop_avoidance(&mut self, context: &Context<'_>) -> Result {
        if self.cmtc_active {
            self.cmtc_stop(context)?;
        }
        if self.quading_active {
            self.quading_stop(context)?;
        }
        if self.pitching_active {
            self.pitching_stop(context)?;
        }
        Ok(())
    }

    fn update(&mut self, context: &Context<'_>) -> Result {
        self.now_ms = millis(context);
        self.current_location = retry(context, || context.ahrs().get_location())?;
        if let Some(location) = self.current_location {
            self.current_altitude = location.alt() as f32 * 0.01;
        }
        self.vehicle_mode = retry(context, || context.vehicle().get_mode())?;
        self.current_target = retry(context, || context.vehicle().get_target_location())?;
        self.current_wp_bearing_deg = retry(context, || context.vehicle().get_wp_bearing_deg())?.unwrap_or(0.0);
        self.current_heading_deg = retry(context, || context.ahrs().get_yaw_rad())? * RAD_TO_DEG;
        let mut velocity = retry(context, || context.ahrs().groundspeed_vector())?;
        self.groundspeed = velocity.length(context)?;
        self.airspeed_current = retry(context, || context.ahrs().airspeed_eas())?.unwrap_or(0.0);

        if matches!(self.vehicle_mode, MODE_AUTO | MODE_GUIDED)
            && !self.pitching_active
            && !self.quading_active
        {
            self.tracker.update(self.current_target);
        }
        self.refresh_parameters()?;
        self.check_activation_switch(context)?;
        if !self.active(context)? {
            return self.stop_avoidance(context);
        }
        self.populate_rangefinders(context)?;

        if !self.quading_active && !self.quading_check(context)? {
            if self.cmtc_enable && !self.cmtc_active {
                let (pitch, altitude) = self.terrain_approaching(context, self.cmtc_height)?;
                if pitch > self.pitch_limit_max_deg * 0.5 {
                    self.cmtc_start(context, altitude)?;
                }
            }
            if !self.cmtc_active {
                let _ = self.pitching_check(context)?;
            }
        }
        if self.terrain_max_exceeded && !self.cmtc_active {
            if self.quading_active {
                self.quading_stop(context)?;
            }
            if self.pitching_active {
                self.pitching_stop(context)?;
            }
        }
        if self.quading_active {
            self.quading_update(context)?;
            return Ok(());
        } else if self.pitching_active {
            self.pitching_update(context)?;
        } else if self.cmtc_active {
            self.cmtc_update(context)?;
        }

        if self.groundspeed_max > 0.0 && self.groundspeed > self.groundspeed_max {
            self.airspeed_desired = self.slow_down(context, self.groundspeed_max - self.groundspeed)?;
        } else if self.groundspeed_airbrake_limit != 0.0 {
            self.slowdown_airbrake_end(context)?;
        }
        self.set_speed(context, self.airspeed_desired)
    }
}

fn run(context: &Context<'_>) -> Result {
    let parameters = Parameters::new(context)?;
    if context.fw_version().r#type()? != VEHICLE_TYPE_PLANE
        || read(&parameters.q_enable)? != 1.0
        || read(&parameters.terrain_enable)? != 1.0
    {
        message(context, 5, format_args!("TerrAvoid: Must run on QuadPlane with terrain follow"))?;
        return Ok(());
    }
    if !retry(context, || context.arming().is_armed())? {
        context.sleep_ms(STARTUP_DELAY_MS)?;
    }
    let mut applet = Applet::new(context, parameters)?;
    message(
        context,
        6,
        format_args!("Terrain Avoid {SCRIPT_VERSION} script loaded"),
    )?;
    loop {
        match applet.update(context) {
            Ok(()) => context.sleep_ms(applet.refresh_ms)?,
            Err(Error::Stopped) => return Err(Error::Stopped),
            Err(Error::Busy) => context.sleep_ms(applet.refresh_ms)?,
            Err(error) => {
                message(context, 0, format_args!("TerrAvoid: Error: {error:?}"))?;
                context.sleep_ms(1_000)?;
            }
        }
    }
}

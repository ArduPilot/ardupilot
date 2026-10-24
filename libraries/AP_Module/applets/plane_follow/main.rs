// SPDX-License-Identifier: GPL-3.0-or-later
//! Native port of AP_Scripting/applets/plane_follow.lua.
#![no_std]
#![no_main]

mod attitude;
mod parameters;
mod pid;

use ap_rust_sdk::{
    enums::{LocationAltFrame, MavResult, MavSeverity, RcChannelAuxFunc},
    CommandIntParams, Context, Error, Location, LogValue, Result, Vector3f,
};
use attitude::AttitudeReceiver;
use core::fmt::{self, Write};
use parameters::{read, retry, Parameters};
use pid::{CrossTrackController, SpeedController};

ap_rust_sdk::module!(run);

const SCRIPT_VERSION: &str = "4.7.0-075";
const VEHICLE_TYPE_PLANE: u8 = 3;
const MODE_GUIDED: u8 = 15;

const MAV_FRAME_GLOBAL: u8 = 0;
const MAV_FRAME_GLOBAL_RELATIVE_ALT: u8 = 3;
const MAV_FRAME_GLOBAL_TERRAIN_ALT: u8 = 10;
const ALT_FRAME_RELATIVE: i32 = 1;
const ALT_FRAME_TERRAIN: i32 = 3;
const SPEED_TYPE_AIRSPEED: f32 = 0.0;
const HEADING_TYPE_HEADING: f32 = 1.0;

const CMD_DO_CHANGE_SPEED: u16 = 178;
const CMD_GUIDED_CHANGE_SPEED: u16 = 43_000;
const CMD_GUIDED_CHANGE_ALTITUDE: u16 = 43_001;
const CMD_GUIDED_CHANGE_HEADING: u16 = 43_002;

const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;
const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

struct Text<const N: usize> {
    bytes: [u8; N],
    length: usize,
}

impl<const N: usize> Text<N> {
    fn new(args: fmt::Arguments<'_>) -> Result<Self> {
        let mut text = Self {
            bytes: [0; N],
            length: 0,
        };
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
    retry(context, || {
        context.gcs().send_text(MavSeverity(severity), text.as_str())
    })
}

fn constrain(mut value: f32, minimum: f32, maximum: f32) -> f32 {
    if value < minimum {
        value = minimum;
    }
    if value > maximum {
        value = maximum;
    }
    value
}

fn millis(context: &Context<'_>) -> u64 {
    context.time_us() / 1_000
}

fn wrap_360(context: &Context<'_>, angle: f32) -> Result<f32> {
    let mut result = retry(context, || context.libm().fmodf(angle, 360.0))?;
    if result < 0.0 {
        result += 360.0;
    }
    Ok(result)
}

fn wrap_180(context: &Context<'_>, angle: f32) -> Result<f32> {
    let result = wrap_360(context, angle)?;
    Ok(if result > 180.0 {
        result - 360.0
    } else {
        result
    })
}

fn follow_frame_to_mavlink(frame: i32) -> u8 {
    match frame {
        ALT_FRAME_TERRAIN => MAV_FRAME_GLOBAL_TERRAIN_ALT,
        ALT_FRAME_RELATIVE => MAV_FRAME_GLOBAL_RELATIVE_ALT,
        _ => MAV_FRAME_GLOBAL,
    }
}

struct Applet<'ctx, 'host> {
    p: Parameters<'ctx, 'host>,
    attitude: AttitudeReceiver,
    distance_pid: SpeedController,
    velocity_pid: SpeedController,
    cross_track_pid: CrossTrackController,

    refresh_rate: f32,
    lost_target_timeout: f32,
    lost_target_countdown: f32,
    overshoot_angle: f32,
    turning_angle: f32,
    lookahead_seconds: f32,
    fail_mode: u8,
    exit_mode: u8,
    foll_sysid: f32,
    foll_ofs_y: f32,
    foll_alt_type: i32,
    use_wide_turns: bool,
    airspeed_min: f32,
    airspeed_max: f32,
    windspeed_max: f32,

    now_ms: u64,
    now_target_heading_ms: u64,
    now_follow_lost_ms: u64,
    now_heading_ms: u64,
    too_close_follow_up: u8,
    save_target_heading1: f32,
    save_target_heading2: f32,

    enabled: bool,
    reported_target: bool,
    lost_target_now_ms: u64,
    last_follow_active_state: Option<u8>,
    telemetry_failed: bool,
    last_telemetry_fail_state: Option<u8>,
}

impl<'ctx, 'host> Applet<'ctx, 'host> {
    fn new(context: &'ctx Context<'host>) -> Result<Self> {
        let p = Parameters::new(context)?;
        let refresh_rate = read(&p.refresh)?;
        let airspeed_min = read(&p.airspeed_min)?;
        let airspeed_max = read(&p.airspeed_max)?;
        let now_ms = millis(context);
        let activation_function = read(&p.activation_function)? as i32;
        let telemetry_function = read(&p.simulate_tlf_function)? as i32;
        let last_follow_active_state = retry(context, || {
            context.rc().get_aux_cached(RcChannelAuxFunc(activation_function))
        })?;
        let last_telemetry_fail_state = retry(context, || {
            context.rc().get_aux_cached(RcChannelAuxFunc(telemetry_function))
        })?;
        let lost_target_timeout = read(&p.timeout)? / refresh_rate;
        let distance_pid = SpeedController::new(
            read(&p.distance_p)?,
            read(&p.distance_i)?,
            read(&p.distance_d)?,
            0.5,
            airspeed_min - airspeed_max,
            airspeed_max - airspeed_min,
        );
        let velocity_pid = SpeedController::new(
            read(&p.velocity_p)?,
            read(&p.velocity_i)?,
            read(&p.velocity_d)?,
            0.5,
            airspeed_min,
            airspeed_max,
        );
        let cross_track_pid = CrossTrackController::new(
            read(&p.cross_track_p)?,
            read(&p.cross_track_i)?,
            read(&p.cross_track_d)?,
            read(&p.cross_track_max)?,
            read(&p.cross_track_i_max)?,
        );
        Ok(Self {
            attitude: AttitudeReceiver::new(context)?,
            refresh_rate,
            lost_target_timeout,
            lost_target_countdown: lost_target_timeout,
            overshoot_angle: read(&p.overshoot_deg)?,
            turning_angle: read(&p.turn_deg)?,
            lookahead_seconds: read(&p.lookahead)?,
            fail_mode: read(&p.fail_mode)? as u8,
            exit_mode: read(&p.exit_mode)? as u8,
            foll_sysid: read(&p.foll_sysid)?,
            foll_ofs_y: read(&p.foll_ofs_y)?,
            foll_alt_type: read(&p.foll_alt_type)? as i32,
            use_wide_turns: read(&p.wide_turns)? != 0.0,
            airspeed_min,
            airspeed_max,
            windspeed_max: read(&p.windspeed_max)?,
            p,
            distance_pid,
            velocity_pid,
            cross_track_pid,
            now_ms,
            now_target_heading_ms: now_ms,
            now_follow_lost_ms: now_ms,
            now_heading_ms: now_ms,
            too_close_follow_up: 0,
            save_target_heading1: -400.0,
            save_target_heading2: -400.0,
            enabled: false,
            reported_target: true,
            lost_target_now_ms: now_ms,
            last_follow_active_state,
            telemetry_failed: false,
            last_telemetry_fail_state,
        })
    }

    fn check_simulated_failure(&mut self, context: &Context<'_>) -> Result {
        let function = read(&self.p.simulate_tlf_function)? as i32;
        let state = retry(context, || {
            context.rc().get_aux_cached(RcChannelAuxFunc(function))
        })?;
        if state != self.last_telemetry_fail_state {
            self.telemetry_failed = state != Some(0);
            self.last_telemetry_fail_state = state;
        }
        Ok(())
    }

    fn enable(&mut self, context: &Context<'_>) -> Result {
        if self.enabled {
            return Ok(());
        }
        if !retry(context, || context.arming().is_armed())? {
            return message(context, 3, format_args!("PFollow: must be armed"));
        }
        let _ = retry(context, || context.vehicle().set_mode(MODE_GUIDED))?;
        self.enabled = true;
        self.lost_target_countdown = self.lost_target_timeout;
        self.distance_pid.reset();
        self.velocity_pid.reset();
        self.cross_track_pid.reset();
        message(context, 6, format_args!("PFollow: enabled"))
    }

    fn disable(&mut self, context: &Context<'_>) -> Result {
        if !self.enabled {
            return Ok(());
        }
        // Preserve the Lua applet's state transition exactly: its condition is
        // inverted relative to the accompanying comment.
        if !retry(context, || context.arming().is_armed())? {
            let _ = retry(context, || context.vehicle().set_mode(self.exit_mode))?;
        }
        self.enabled = false;
        message(context, 6, format_args!("PFollow: disabled"))
    }

    fn check_follow_switch(&mut self, context: &Context<'_>) -> Result {
        let function = read(&self.p.activation_function)? as i32;
        let state = retry(context, || {
            context.rc().get_aux_cached(RcChannelAuxFunc(function))
        })?;
        if state == self.last_follow_active_state {
            return Ok(());
        }
        match state {
            Some(0) => self.disable(context)?,
            Some(2) => self.enable(context)?,
            _ => {}
        }
        self.last_follow_active_state = state;
        Ok(())
    }

    fn follow_active(&mut self, context: &Context<'_>) -> Result<bool> {
        if !self.enabled {
            return Ok(false);
        }
        if retry(context, || context.vehicle().get_mode())? != MODE_GUIDED {
            self.reported_target = false;
            return Ok(false);
        }
        if retry(context, || context.follow().have_target())? {
            self.reported_target = true;
            self.lost_target_now_ms = self.now_ms;
        } else {
            if self.reported_target && self.now_ms.saturating_sub(self.lost_target_now_ms) > 5_000 {
                let sysid = retry(context, || context.follow().get_target_sysid())?;
                message(
                    context,
                    4,
                    format_args!("PFollow: lost prior SYSID: {sysid}"),
                )?;
                self.lost_target_now_ms = self.now_ms;
            }
            self.reported_target = false;
            let sysid = retry(context, || context.follow().get_target_sysid())?;
            message(
                context,
                4,
                format_args!("PFollow: no target SYSID: {sysid}"),
            )?;
        }
        Ok(self.reported_target)
    }

    fn set_vehicle_altitude(&self, context: &Context<'_>, altitude: f32, frame: i32) -> Result {
        let result = retry(context, || {
            context.gcs().run_command_int(
                CMD_GUIDED_CHANGE_ALTITUDE,
                &CommandIntParams {
                    p3: 1_000.0,
                    z: altitude,
                    frame: follow_frame_to_mavlink(frame),
                    ..Default::default()
                },
            )
        })?;
        if result != MavResult(0) {
            message(
                context,
                3,
                format_args!("PFollow: MAVLink CHANGE_ALTITUDE returned false"),
            )?;
        }
        Ok(())
    }

    fn set_vehicle_heading(&self, context: &Context<'_>, heading: f32) -> Result {
        if heading <= -400.0 || heading > 360.0 {
            return message(
                context,
                3,
                format_args!("PFollow: set_vehicle_heading no heading"),
            );
        }
        let result = retry(context, || {
            context.gcs().run_command_int(
                CMD_GUIDED_CHANGE_HEADING,
                &CommandIntParams {
                    p1: HEADING_TYPE_HEADING,
                    p2: heading,
                    p3: 10.0,
                    frame: MAV_FRAME_GLOBAL,
                    ..Default::default()
                },
            )
        })?;
        if result != MavResult(0) {
            message(
                context,
                3,
                format_args!("PFollow: MAVLink GUIDED_CHANGE_HEADING failed"),
            )?;
        }
        Ok(())
    }

    fn set_vehicle_speed(&self, context: &Context<'_>, speed: f32) -> Result {
        let guided = retry(context, || context.vehicle().get_mode())? == MODE_GUIDED;
        let command = if guided {
            CMD_GUIDED_CHANGE_SPEED
        } else {
            CMD_DO_CHANGE_SPEED
        };
        let result = retry(context, || {
            context.gcs().run_command_int(
                command,
                &CommandIntParams {
                    p1: SPEED_TYPE_AIRSPEED,
                    p2: speed,
                    p3: 0.0,
                    frame: MAV_FRAME_GLOBAL,
                    ..Default::default()
                },
            )
        })?;
        if result != MavResult(0) {
            let name = if guided {
                "GUIDED_CHANGE_SPEED"
            } else {
                "DO_CHANGE_SPEED"
            };
            message(context, 3, format_args!("PFollow: MAVLink {name} failed"))?;
        }
        Ok(())
    }

    fn calculate_airspeed(
        &self,
        context: &Context<'_>,
        velocity: Vector3f,
        wind: Vector3f,
        eas2tas: f32,
    ) -> Result<f32> {
        let airspeed_vector = velocity.sub(context, wind)?;
        let mut airspeed = airspeed_vector.length(context)? * eas2tas;
        airspeed = constrain(
            airspeed,
            airspeed - self.windspeed_max,
            airspeed + self.windspeed_max,
        );
        Ok(airspeed / eas2tas)
    }

    fn calculate_track_distance(
        &self,
        context: &Context<'_>,
        mut follower: Location,
        mut leader: Location,
    ) -> Result<(f32, f32)> {
        let Some(velocity) = retry(context, || context.ahrs().get_velocity_ned())? else {
            return Ok((0.0, 0.0));
        };
        if velocity.x == 0.0 && velocity.y == 0.0 {
            return Ok((0.0, 0.0));
        }
        let mut direction = Vector3f::new(velocity.x, velocity.y, 0.0);
        direction.normalize(context)?;
        let relative = follower.get_distance_ned(context, &mut leader)?;
        let along = relative.dot(context, direction)?;
        let mut perpendicular = Vector3f::new(-direction.y, direction.x, 0.0);
        perpendicular.normalize(context)?;
        Ok((along, -relative.dot(context, perpendicular)?))
    }

    fn calculate_target_angle(&mut self, context: &Context<'_>, heading: f32) -> Result<f32> {
        let mut angle = 0.0;
        if heading > -400.0 {
            let angle1 = wrap_180(context, (self.save_target_heading1 - heading).abs())?;
            let angle2 = wrap_180(context, (self.save_target_heading2 - heading).abs())?;
            angle = if angle2 > angle1 { angle2 } else { angle1 };
            if self.now_ms.saturating_sub(self.now_target_heading_ms) > 1_000 {
                self.save_target_heading2 = self.save_target_heading1;
                self.save_target_heading1 = heading;
                self.now_target_heading_ms = self.now_ms;
            }
        }
        Ok(angle)
    }

    fn handle_lost_target(&mut self, context: &Context<'_>) -> Result {
        self.lost_target_countdown -= 1.0;
        if self.lost_target_countdown <= 0.0 {
            self.enabled = false;
            let _ = retry(context, || context.vehicle().set_mode(self.fail_mode))?;
            message(
                context,
                3,
                format_args!("PFollow: follow: {:.0} FAILED", self.foll_sysid),
            )?;
            return Ok(());
        }
        if self.now_ms.saturating_sub(self.now_follow_lost_ms) > 2_000 {
            message(
                context,
                4,
                format_args!(
                    "PFollow: follow: lost {:.0} set hdg: {:.0}",
                    self.foll_sysid, self.save_target_heading1
                ),
            )?;
            self.now_follow_lost_ms = self.now_ms;
            self.set_vehicle_heading(context, self.save_target_heading1)?;
        }
        Ok(())
    }

    fn update(&mut self, context: &Context<'_>) -> Result {
        self.now_ms = millis(context);
        let eas2tas = retry(context, || context.ahrs().get_eas2_tas())?;
        let wind = retry(context, || context.ahrs().get_wind())?.unwrap_or_default();

        self.check_simulated_failure(context)?;
        self.check_follow_switch(context)?;
        if !self.follow_active(context)? {
            return Ok(());
        }

        let close_distance = read(&self.p.distance_close)?;
        let long_distance = close_distance * 4.0;
        let altitude_override = read(&self.p.altitude_override)?;
        self.lost_target_timeout = read(&self.p.timeout)? / self.refresh_rate;
        self.overshoot_angle = read(&self.p.overshoot_deg)?;
        self.turning_angle = read(&self.p.turn_deg)?;
        self.foll_ofs_y = read(&self.p.foll_ofs_y)?;
        self.foll_alt_type = read(&self.p.foll_alt_type)? as i32;
        self.use_wide_turns = read(&self.p.wide_turns)? != 0.0;

        let Some(mut current_location) = retry(context, || context.ahrs().get_location())? else {
            return Ok(());
        };
        let current_altitude = current_location.alt() as f32 * 0.01;
        let vehicle_airspeed =
            retry(context, || context.ahrs().airspeed_eas())?.ok_or(Error::Unavailable)?;
        let current_target = retry(context, || context.vehicle().get_target_location())?;
        let target = retry(context, || {
            context.follow().get_target_location_and_velocity()
        })?;
        let target_offset = retry(context, || {
            context.follow().get_target_location_and_velocity_ofs()
        })?;
        let target_heading =
            retry(context, || context.follow().get_target_heading_deg())?.unwrap_or(-400.0);

        let (
            Some((_target_location, target_velocity)),
            Some((mut target_location_offset, _target_velocity_offset)),
            Some(_),
        ) = (target, target_offset, current_target)
        else {
            self.handle_lost_target(context)?;
            return Ok(());
        };
        if self.telemetry_failed {
            self.handle_lost_target(context)?;
            return Ok(());
        }
        self.lost_target_countdown = self.lost_target_timeout;
        self.now_follow_lost_ms = self.now_ms;

        let mut offset_for_distance = target_location_offset;
        let mut target_distance_rotated =
            current_location.get_distance_ned(context, &mut offset_for_distance)?;
        let (along_track_distance, cross_track_distance) =
            self.calculate_track_distance(context, current_location, target_location_offset)?;
        let target_airspeed = self.calculate_airspeed(context, target_velocity, wind, eas2tas)?;
        let vehicle_heading = wrap_360(
            context,
            retry(context, || context.ahrs().get_yaw_rad())? * RAD_TO_DEG,
        )?
        .abs();
        let heading_to_target_offset =
            current_location.get_bearing(context, &mut target_location_offset)? * RAD_TO_DEG;
        let offset_angle = wrap_180(context, vehicle_heading - heading_to_target_offset)?;
        target_distance_rotated.rotate_xy(context, vehicle_heading * DEG_TO_RAD)?;

        let projected_distance =
            along_track_distance - (vehicle_airspeed - target_airspeed) * self.lookahead_seconds;
        let target_angle = self.calculate_target_angle(context, target_heading)?;
        let mut turning =
            projected_distance.abs() < long_distance && target_angle.abs() > self.turning_angle;
        let turn_starting = false;
        let target_attitude = self.attitude.get(context, self.foll_sysid as u32)?;
        let pre_roll_target_heading = target_heading;
        let mut turning_airspeed_ratio = 1.0;
        let mut tight_turn = false;
        if let Some(attitude) = target_attitude {
            // Preserve Lua's `and`/`or` precedence: rollspeed alone is enough
            // to enter this branch even when the corrected timestamp is old.
            let fresh_roll = (self.now_ms as i64 - attitude.timestamp_ms)
                < self.lost_target_timeout as i64
                && attitude.roll.abs() > 0.1;
            if target_airspeed > self.airspeed_min && (fresh_roll || attitude.rollspeed.abs() > 1.0)
            {
                let rollspeed_impact = attitude.rollspeed * (attitude.delay_ms + 1_000.0);
                let tangent = retry(context, || {
                    context.libm().tanf(attitude.roll + rollspeed_impact)
                })?;
                let target_turn_radius = vehicle_airspeed * vehicle_airspeed / (9.80665 * tangent);
                turning = true;
                if (attitude.roll < 0.0 && self.foll_ofs_y < 0.0)
                    || (attitude.roll > 0.0 && self.foll_ofs_y > 0.0)
                {
                    tight_turn = true;
                }
                let mut turn_radius = target_turn_radius + self.foll_ofs_y;
                if tight_turn {
                    turn_radius = target_turn_radius - self.foll_ofs_y;
                }
                let my_airspeed = (target_airspeed / target_turn_radius) * turn_radius;
                if my_airspeed > 0.0 && my_airspeed > self.airspeed_min && target_airspeed > 0.0 {
                    turning_airspeed_ratio = constrain(my_airspeed / target_airspeed, 0.0, 2.0);
                }
            }
        }

        let mut too_close = projected_distance > 0.0
            && projected_distance.abs() < close_distance
            && offset_angle < self.overshoot_angle;
        let overshot = !too_close
            && (((projected_distance < 0.0 || along_track_distance < 0.0)
                && along_track_distance.abs() < close_distance)
                || offset_angle > self.overshoot_angle);
        if overshot || too_close || self.too_close_follow_up > 0 {
            if self.too_close_follow_up > 0 {
                too_close = true;
                self.too_close_follow_up -= 1;
            } else {
                self.too_close_follow_up = 10;
            }
        } else {
            self.too_close_follow_up = 0;
        }

        let (target_altitude, frame_type_log) = if altitude_override != 0.0 {
            (altitude_override, -1)
        } else {
            let _ = target_location_offset
                .change_alt_frame(context, LocationAltFrame(self.foll_alt_type))?;
            (
                target_location_offset.alt() as f32 * 0.01,
                self.foll_alt_type,
            )
        };

        let close = projected_distance.abs() < close_distance;
        let too_wide = target_distance_rotated.y.abs() > close_distance / 4.0 && !turning;
        let mut desired_heading = target_heading;
        let mechanism;
        if (along_track_distance.abs() < self.airspeed_max * 0.75
            || cross_track_distance.abs() < self.airspeed_max * 0.25)
            || (turning
                && ((tight_turn && turn_starting) || self.use_wide_turns || self.foll_ofs_y == 0.0))
            || ((close || overshot) && !too_wide)
        {
            mechanism = 2;
        } else {
            desired_heading = heading_to_target_offset;
            mechanism = 1;
        }
        if close || self.too_close_follow_up > 0 {
            let dt = self.now_ms.saturating_sub(self.now_heading_ms) as f32 * 0.001;
            desired_heading =
                self.cross_track_pid
                    .compute(desired_heading, cross_track_distance, dt);
        }

        let mut distance_velocity_error = along_track_distance * self.refresh_rate * 2.0;
        if distance_velocity_error < 0.0 {
            distance_velocity_error *= 5.0;
        }
        if turning_airspeed_ratio > 0.0 && turning_airspeed_ratio < 2.0 {
            distance_velocity_error *= turning_airspeed_ratio;
        }
        let airspeed_error = target_airspeed - vehicle_airspeed;
        let distance_velocity =
            self.distance_pid
                .update(context, airspeed_error, distance_velocity_error)?;
        let airspeed_new =
            self.velocity_pid
                .update(context, vehicle_airspeed, distance_velocity)?;

        self.set_vehicle_heading(context, desired_heading)?;
        self.set_vehicle_altitude(context, target_altitude, self.foll_alt_type)?;
        self.set_vehicle_speed(
            context,
            constrain(airspeed_new, self.airspeed_min, self.airspeed_max),
        )?;

        context.logger().write(
            "PF1",
            "Dst,DstP,DstE,AspT,Asp,AspD,AspE,AspO,TrnR,Mech,Cls,CF,OS",
            Some(("mmmnnnnnn----", "-------------")),
            &[
                LogValue::Float(along_track_distance),
                LogValue::Float(projected_distance),
                LogValue::Float(distance_velocity_error),
                LogValue::Float(target_airspeed),
                LogValue::Float(vehicle_airspeed),
                LogValue::Float(distance_velocity),
                LogValue::Float(airspeed_error),
                LogValue::Float(airspeed_new),
                LogValue::Float(turning_airspeed_ratio),
                LogValue::U8(mechanism),
                LogValue::U8(u8::from(too_close)),
                // Lua considers numeric zero truthy, so this field is always one.
                LogValue::U8(1),
                LogValue::U8(u8::from(overshot)),
            ],
        )?;
        context.logger().write(
            "PF2",
            "AngT,AngO,Alt,AltT,AltFrm,HdgT,Hdg,HdgP,HdgO,XTD",
            Some(("ddmm-ddddm", "----------")),
            &[
                LogValue::Float(target_angle),
                LogValue::Float(offset_angle),
                LogValue::Float(current_altitude),
                LogValue::Float(target_altitude),
                LogValue::I8(frame_type_log as i8),
                LogValue::Float(target_heading),
                LogValue::Float(vehicle_heading),
                LogValue::Float(pre_roll_target_heading),
                LogValue::Float(desired_heading),
                LogValue::Float(cross_track_distance),
            ],
        )?;
        Ok(())
    }
}

fn run(context: &Context<'_>) -> Result {
    let mut applet = Applet::new(context)?;
    if retry(context, || context.fw_version().r#type())? != VEHICLE_TYPE_PLANE {
        message(context, 3, format_args!("PFollow: must run on Plane"))?;
        return Ok(());
    }
    if !retry(context, || context.arming().is_armed())? {
        context.sleep_ms(25_000)?;
    }
    message(
        context,
        6,
        format_args!("Plane Follow {SCRIPT_VERSION} script loaded"),
    )?;

    loop {
        match applet.update(context) {
            Ok(()) => context.sleep_ms((applet.refresh_rate * 1_000.0) as u32)?,
            Err(Error::Stopped) => return Err(Error::Stopped),
            Err(Error::Busy) => context.sleep_ms((applet.refresh_rate * 1_000.0) as u32)?,
            Err(error) => {
                message(
                    context,
                    1,
                    format_args!("PFollow Internal Error: {error:?}"),
                )?;
                context.sleep_ms(1_000)?;
            }
        }
    }
}

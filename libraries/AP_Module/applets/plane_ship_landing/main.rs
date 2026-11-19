// SPDX-License-Identifier: GPL-3.0-or-later
//! Native port of AP_Scripting/applets/plane_ship_landing.lua.
#![no_std]
#![no_main]

mod parameters;

use ap_rust_sdk::{
    enums::{LocationAltFrame, MavSeverity},
    Context, Error, Location, LogValue, Result, Vector2f, Vector3f,
};
use core::fmt::{self, Write};
use parameters::{read, retry, save, Parameters};

ap_rust_sdk::module!(run);

const MODE_MANUAL: u8 = 0;
const MODE_RTL: u8 = 11;
const MODE_AUTO: u8 = 10;
const MODE_QLOITER: u8 = 19;
const MODE_QRTL: u8 = 21;
const NAV_TAKEOFF: u16 = 22;
const NAV_VTOL_TAKEOFF: u16 = 84;
const ALT_FRAME_ABSOLUTE: LocationAltFrame = LocationAltFrame(0);

const THROTTLE_LOW: u8 = 0;
const THROTTLE_MID: u8 = 1;
const THROTTLE_HIGH: u8 = 2;

const STAGE_HOLDOFF: u8 = 0;
const STAGE_DESCEND: u8 = 1;
const STAGE_APPROACH: u8 = 2;
const STAGE_IDLE: u8 = STAGE_APPROACH;

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

fn wrap_360(context: &Context<'_>, angle: f32) -> Result<f32> {
    let mut value = retry(context, || context.libm().fmodf(angle, 360.0))?;
    if value < 0.0 {
        value += 360.0;
    }
    Ok(value)
}

fn wrap_180(context: &Context<'_>, angle: f32) -> Result<f32> {
    let value = wrap_360(context, angle)?;
    Ok(if value > 180.0 { value - 360.0 } else { value })
}

struct Applet<'ctx, 'host> {
    p: Parameters<'ctx, 'host>,
    auth_id: u8,
    target_pos: Location,
    current_pos: Location,
    target_velocity: Vector3f,
    target_heading: f32,
    landing_stage: u8,
    vehicle_mode: u8,
    reached_alt: bool,
    throttle_pos: u8,
    have_target: bool,
}

impl<'ctx, 'host> Applet<'ctx, 'host> {
    fn new(context: &'ctx Context<'host>) -> Result<Self> {
        let p = Parameters::new(context)?;
        let auth_id =
            retry(context, || context.arming().get_aux_auth_id())?.ok_or(Error::NoMemory)?;
        retry(context, || {
            context.arming().set_aux_auth_failed(auth_id, "Ship: no beacon")
        })?;
        let applet = Self {
            p,
            auth_id,
            target_pos: Location::default(),
            current_pos: Location::default(),
            target_velocity: Vector3f::default(),
            target_heading: 0.0,
            landing_stage: STAGE_HOLDOFF,
            vehicle_mode: MODE_MANUAL,
            reached_alt: false,
            throttle_pos: THROTTLE_HIGH,
            have_target: false,
        };
        applet.check_parameters(context)?;
        Ok(applet)
    }

    fn check_parameter(
        &self,
        context: &Context<'_>,
        name: &str,
        parameter: &ap_rust_sdk::Parameter<'_, '_>,
        wanted: f32,
    ) -> Result {
        let current = read(parameter)?;
        if (wanted - current).abs() > 0.001 {
            save(parameter, wanted)?;
            message(
                context,
                6,
                format_args!("Parameter {name} set to {wanted:.2} was {current:.2}"),
            )?;
        }
        Ok(())
    }

    fn check_parameters(&self, context: &Context<'_>) -> Result {
        self.check_parameter(context, "FOLL_ENABLE", &self.p.foll_enable, 1.0)?;
        self.check_parameter(context, "FOLL_OFS_TYPE", &self.p.foll_ofs_type, 1.0)?;
        self.check_parameter(context, "FOLL_ALT_TYPE", &self.p.foll_alt_type, 0.0)
    }

    fn get_land_airspeed(&self) -> Result<f32> {
        let landing = read(&self.p.tecs_land_airspeed)?;
        if landing < 0.0 {
            read(&self.p.airspeed_cruise)
        } else {
            Ok(landing)
        }
    }

    fn stopping_distance(&self, context: &Context<'_>) -> Result<f32> {
        let tas = self.get_land_airspeed()? * retry(context, || context.ahrs().get_eas2_tas())?;
        let mut wind3 = retry(context, || context.ahrs().get_wind())?.unwrap_or_default();
        let mut wind = wind3.xy(context)?;
        let approach_angle = -(self.target_heading + read(&self.p.land_angle)?) * DEG_TO_RAD;
        wind.rotate(context, approach_angle)?;
        let mut velocity = self.target_velocity;
        let mut ship = velocity.xy(context)?;
        ship.rotate(context, approach_angle)?;

        let tas_sq = tas * tas;
        let y_sq = wind.y * wind.y;
        let mut closing_speed = if tas_sq >= y_sq {
            retry(context, || context.libm().sqrtf(tas_sq - y_sq))?
        } else {
            1.0
        };
        closing_speed += wind.x;
        closing_speed -= ship.x;
        Ok((closing_speed * closing_speed) / (2.0 * read(&self.p.q_transition_decel)?))
    }

    fn holdoff_radius(&self) -> Result<f32> {
        let rtl_radius = read(&self.p.rtl_radius)?;
        if rtl_radius != 0.0 {
            Ok(rtl_radius)
        } else {
            read(&self.p.wp_loiter_radius)
        }
    }

    fn holdoff_distance(&self, context: &Context<'_>) -> Result<f32> {
        let radius = self.holdoff_radius()?;
        let holdoff = (radius * 1.5).abs();
        let stop = self.stopping_distance(context)?;
        let increased = if holdoff * 2.5 < stop * 2.0 {
            holdoff * 2.5
        } else {
            stop * 2.0
        };
        Ok(if holdoff > increased {
            holdoff
        } else {
            increased
        })
    }

    fn holdoff_position(&mut self, context: &Context<'_>) -> Result<Location> {
        let radius = self.holdoff_radius()?;
        let heading = self.target_heading + read(&self.p.land_angle)?;
        let distance = self.holdoff_distance(context)?;
        let mut offset = Vector2f {
            x: -distance,
            y: radius,
        };
        offset.rotate(context, heading * DEG_TO_RAD)?;
        let mut target = self.target_pos.copy(context)?;
        target.offset(context, offset.x, offset.y)?;
        Ok(target)
    }

    fn target_altitude(&self) -> Result<f32> {
        let base = self.target_pos.alt() as f32 * 0.01;
        if self.landing_stage == STAGE_HOLDOFF {
            Ok(base + read(&self.p.rtl_altitude)?)
        } else {
            Ok(base + read(&self.p.q_rtl_alt)?)
        }
    }

    fn update_throttle_position(&mut self, context: &Context<'_>) -> Result {
        let position = if !retry(context, || context.rc().has_valid_input())? {
            THROTTLE_LOW
        } else {
            let channel_number = read(&self.p.rcmap_throttle)? as u8;
            let channel = retry(context, || context.rc().get_channel(channel_number))?
                .ok_or(Error::Unavailable)?;
            let input = retry(context, || channel.norm_input_ignore_trim(context))?;
            let throttle = (input + 1.0) * 0.5;
            if throttle >= 0.40 {
                THROTTLE_HIGH
            } else if throttle >= 0.1 {
                THROTTLE_MID
            } else {
                THROTTLE_LOW
            }
        };

        if position != self.throttle_pos {
            self.reached_alt = false;
            if self.landing_stage == STAGE_HOLDOFF && position <= THROTTLE_MID {
                self.landing_stage = STAGE_DESCEND;
                message(
                    context,
                    6,
                    format_args!(
                        "Descending for approach (hd={:.1}m h={:.1} th={:.1})",
                        self.holdoff_distance(context)?,
                        self.current_pos.alt() as f32 * 0.01,
                        self.target_altitude()?
                    ),
                )?;
            }
            if self.landing_stage == STAGE_DESCEND && position == THROTTLE_HIGH {
                message(context, 6, format_args!("Climbing for holdoff"))?;
                self.landing_stage = STAGE_HOLDOFF;
            }
        }
        self.throttle_pos = position;
        Ok(())
    }

    fn update_mode(&mut self, context: &Context<'_>) -> Result {
        let mode = retry(context, || context.vehicle().get_mode())?;
        if mode == self.vehicle_mode {
            return Ok(());
        }
        self.vehicle_mode = mode;
        if mode == MODE_RTL {
            self.landing_stage = STAGE_HOLDOFF;
            self.reached_alt = false;
        } else if mode != MODE_QRTL {
            self.landing_stage = STAGE_IDLE;
            self.reached_alt = false;
        }
        Ok(())
    }

    fn update_target(&mut self, context: &Context<'_>) -> Result {
        if !retry(context, || context.follow().have_target())? {
            if self.have_target {
                message(context, 4, format_args!("Lost beacon"))?;
                retry(context, || {
                    context.arming().set_aux_auth_failed(self.auth_id, "Ship: no beacon")
                })?;
            }
            self.have_target = false;
            return Ok(());
        }

        let Some((position, velocity)) = retry(context, || {
            context.follow().get_target_location_and_velocity_ofs()
        })?
        else {
            if self.have_target {
                message(context, 4, format_args!("No target location/velocity"))?;
                self.have_target = false;
            }
            return Ok(());
        };
        self.target_pos = position;
        self.target_velocity = velocity;
        if !self.have_target {
            message(context, 6, format_args!("Have beacon"))?;
            retry(context, || context.arming().set_aux_auth_passed(self.auth_id))?;
        }
        self.have_target = true;
        let _ = self
            .target_pos
            .change_alt_frame(context, ALT_FRAME_ABSOLUTE)?;
        self.target_heading = retry(context, || context.follow().get_target_heading_deg())?
            .ok_or(Error::Unavailable)?;
        self.target_velocity.z = 0.0;
        Ok(())
    }

    fn update_altitude(&mut self, context: &Context<'_>) -> Result {
        let altitude = self.current_pos.alt() as f32 * 0.01;
        let target = self.target_altitude()?;
        if (self.landing_stage == STAGE_HOLDOFF || self.landing_stage == STAGE_DESCEND)
            && (altitude - target).abs() < 3.0
        {
            if !self.reached_alt {
                message(context, 6, format_args!("Reached target altitude"))?;
            }
            self.reached_alt = true;
        }
        Ok(())
    }

    fn update_auto_offset(&mut self, context: &Context<'_>) -> Result {
        if retry(context, || context.arming().is_armed())?
            || retry(context, || context.libm().floorf(read(&self.p.auto_offset)?))? != 1.0
        {
            return Ok(());
        }
        let Some((mut target, _velocity)) = retry(context, || {
            context.follow().get_target_location_and_velocity()
        })?
        else {
            return Err(Error::Unavailable);
        };
        let _ = target.change_alt_frame(context, ALT_FRAME_ABSOLUTE)?;
        let mut current = self.current_pos;
        let mut offset = target.get_distance_ned(context, &mut current)?;
        offset.rotate_xy(context, -self.target_heading * DEG_TO_RAD)?;
        message(
            context,
            6,
            format_args!(
                "Set follow offset ({:.2},{:.2},{:.2})",
                offset.x, offset.y, offset.z
            ),
        )?;
        save(&self.p.foll_ofs_x, offset.x)?;
        save(&self.p.foll_ofs_y, offset.y)?;
        save(&self.p.foll_ofs_z, offset.z)?;
        save(&self.p.auto_offset, 0.0)
    }

    fn check_approach_tangent(&mut self, context: &Context<'_>) -> Result {
        let mut current = self.current_pos;
        let mut target = self.target_pos;
        let distance = current.get_distance(context, &mut target)?;
        let holdoff = self.holdoff_distance(context)?;
        if self.landing_stage == STAGE_HOLDOFF
            && self.throttle_pos <= THROTTLE_MID
            && distance < 4.0 * holdoff
        {
            message(
                context,
                6,
                format_args!("Descending for approach (hd={holdoff:.1}m)"),
            )?;
            self.landing_stage = STAGE_DESCEND;
        }
        if self.reached_alt && self.landing_stage == STAGE_DESCEND {
            let target_bearing = wrap_180(
                context,
                current.get_bearing(context, &mut target)? * RAD_TO_DEG,
            )?;
            let mut ground_velocity = retry(context, || context.ahrs().groundspeed_vector())?;
            let ground_bearing = wrap_180(context, ground_velocity.angle(context)? * RAD_TO_DEG)?;
            let margin = 10.0;
            let error1 = wrap_180(context, target_bearing - ground_bearing)?.abs();
            let error2 = wrap_180(
                context,
                ground_bearing - (self.target_heading + read(&self.p.land_angle)?),
            )?
            .abs();
            context.logger().write(
                "SLND",
                "TBrg,GBrg,Dist,HDist,Err1,Err2",
                None,
                &[
                    LogValue::Float(target_bearing),
                    LogValue::Float(ground_bearing),
                    LogValue::Float(distance),
                    LogValue::Float(holdoff),
                    LogValue::Float(error1),
                    LogValue::Float(error2),
                ],
            )?;
            if error1 < margin
                && distance < 2.5 * holdoff
                && distance > 0.7 * holdoff
                && error2 < 2.0 * margin
            {
                message(context, 6, format_args!("Starting approach"))?;
                self.landing_stage = STAGE_APPROACH;
                let _ = retry(context, || context.vehicle().set_mode(MODE_QRTL))?;
            }
        }
        Ok(())
    }

    fn check_approach_abort(&mut self, context: &Context<'_>) -> Result {
        if self.current_pos.alt() as f32 * 0.01 > self.target_altitude()? {
            message(context, 5, format_args!("Aborting landing"))?;
            self.landing_stage = STAGE_HOLDOFF;
            let _ = retry(context, || context.vehicle().set_mode(MODE_RTL))?;
        }
        Ok(())
    }

    fn update(&mut self, context: &Context<'_>) -> Result {
        if read(&self.p.enable)? < 1.0 {
            return Ok(());
        }
        self.update_target(context)?;
        if !self.have_target {
            return Ok(());
        }
        let Some(position) = retry(context, || context.ahrs().get_position())? else {
            return Ok(());
        };
        self.current_pos = position;
        let _ = self
            .current_pos
            .change_alt_frame(context, ALT_FRAME_ABSOLUTE)?;
        let Some(mut next_waypoint) = retry(context, || context.vehicle().get_target_location())?
        else {
            return Ok(());
        };

        self.update_throttle_position(context)?;
        self.update_mode(context)?;
        self.update_altitude(context)?;
        self.update_auto_offset(context)?;
        let _ = retry(context, || context.ahrs().set_home(&mut self.target_pos))?;
        let _ = next_waypoint.change_alt_frame(context, ALT_FRAME_ABSOLUTE)?;

        if self.vehicle_mode == MODE_RTL {
            let mut holdoff = self.holdoff_position(context)?;
            let _ = holdoff.change_alt_frame(context, ALT_FRAME_ABSOLUTE)?;
            let altitude_cm = retry(context, || {
                context.libm().floorf(self.target_altitude()? * 100.0)
            })? as i32;
            holdoff.set_alt(context, altitude_cm)?;
            let _ = retry(context, || {
                context.vehicle().update_target_location(&mut next_waypoint, &mut holdoff)
            })?;
            if self.throttle_pos == THROTTLE_LOW {
                self.check_approach_tangent(context)?;
            }
        } else if self.vehicle_mode == MODE_QRTL {
            let mut velocity = self.target_velocity;
            let mut velocity_xy = velocity.xy(context)?;
            let _ = retry(context, || {
                context.vehicle().set_velocity_match(&mut velocity_xy)
            })?;
            self.target_pos.set_alt(context, next_waypoint.alt())?;
            let _ = retry(context, || {
                context.vehicle().update_target_location(&mut next_waypoint, &mut self.target_pos)
            })?;
            if self.throttle_pos == THROTTLE_HIGH {
                self.check_approach_abort(context)?;
            }
        } else if self.vehicle_mode == MODE_AUTO {
            let nav_id = retry(context, || context.mission().get_current_nav_id())?;
            if nav_id == NAV_VTOL_TAKEOFF || nav_id == NAV_TAKEOFF {
                let mut velocity = self.target_velocity;
                let mut velocity_xy = velocity.xy(context)?;
                let _ = retry(context, || {
                    context.vehicle().set_velocity_match(&mut velocity_xy)
                })?;
                let mut current = self.current_pos;
                let mut takeoff_position = current.copy(context)?;
                takeoff_position.set_alt(context, next_waypoint.alt())?;
                let _ = retry(context, || {
                    context
                        .vehicle().update_target_location(&mut next_waypoint, &mut takeoff_position)
                })?;
            }
        } else if self.vehicle_mode == MODE_QLOITER {
            let mut velocity = self.target_velocity;
            let mut velocity_xy = velocity.xy(context)?;
            let _ = retry(context, || {
                context.vehicle().set_velocity_match(&mut velocity_xy)
            })?;
        }
        Ok(())
    }
}

fn run(context: &Context<'_>) -> Result {
    let mut applet = Applet::new(context)?;
    message(context, 6, format_args!("ShipLanding: loaded"))?;
    loop {
        match applet.update(context) {
            Ok(()) => context.sleep_ms(50)?,
            Err(Error::Stopped) => return Err(Error::Stopped),
            Err(Error::Busy) => context.sleep_ms(50)?,
            Err(error) => {
                message(context, 3, format_args!("Internal Error: {error:?}"))?;
                context.sleep_ms(1_000)?;
            }
        }
    }
}

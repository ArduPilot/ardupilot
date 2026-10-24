// SPDX-License-Identifier: GPL-3.0-or-later
//! Native counterpart of AP_Scripting/applets/plane_precland.lua.
#![no_std]
#![no_main]

use ap_rust_sdk::{
    enums::{MavSeverity, RcChannelAuxFunc, Rotation},
    Context, Error, LogValue, Parameter, Result, Vector2f,
};
use core::fmt::{self, Write};

ap_rust_sdk::module!(run);

const PARAM_TABLE_KEY: u8 = 12;
const MODE_QLOITER: u8 = 19;
const MODE_QLAND: u8 = 20;
const AUX_PRECLOITER: RcChannelAuxFunc = RcChannelAuxFunc(39);
const RANGEFINDER_DOWN: Rotation = Rotation(25);

struct Text<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> Text<N> {
    fn new(args: fmt::Arguments<'_>) -> Result<Self> {
        let mut text = Self {
            bytes: [0; N],
            len: 0,
        };
        text.write_fmt(args).map_err(|_| Error::Invalid)?;
        Ok(text)
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap()
    }
}

impl<const N: usize> Write for Text<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        if value.len() > N - self.len {
            return Err(fmt::Error);
        }
        self.bytes[self.len..self.len + value.len()].copy_from_slice(value.as_bytes());
        self.len += value.len();
        Ok(())
    }
}

fn retry<T>(context: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
    let start = context.time_us();
    loop {
        match call() {
            Err(Error::Busy) if context.time_us().saturating_sub(start) < 100_000 => {
                context.sleep_ms(1)?;
            }
            result => return result,
        }
    }
}

fn message(context: &Context<'_>, severity: i32, text: &str) -> Result {
    match context.gcs().send_text(MavSeverity(severity), text) {
        Err(Error::Stopped) => Err(Error::Stopped),
        _ => Ok(()),
    }
}

fn parameter<'a, 'host>(context: &'a Context<'host>, name: &str) -> Result<Parameter<'a, 'host>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Invalid)
}

fn add_parameter<'a, 'host>(
    context: &'a Context<'host>,
    index: u8,
    name: &str,
    default: f32,
) -> Result<Parameter<'a, 'host>> {
    if !retry(context, || {
        context.param().add_param(PARAM_TABLE_KEY, index, name, default)
    })? {
        return Err(Error::Host);
    }
    let full_name = Text::<17>::new(format_args!("PLND_{name}"))?;
    parameter(context, full_name.as_str())
}

fn value(parameter: &Parameter<'_, '_>) -> Result<f32> {
    let value = parameter.get()?.ok_or(Error::Invalid)?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(Error::Invalid)
    }
}

struct Parameters<'a, 'host> {
    altitude_cutoff: Parameter<'a, 'host>,
    distance_cutoff: Parameter<'a, 'host>,
    enabled: Parameter<'a, 'host>,
    xy_distance_max: Parameter<'a, 'host>,
    options: Parameter<'a, 'host>,
}

impl<'a, 'host> Parameters<'a, 'host> {
    fn new(context: &'a Context<'host>) -> Result<Self> {
        if !retry(context, || {
            context.param().add_table(PARAM_TABLE_KEY, "PLND_", 10)
        })? {
            return Err(Error::Host);
        }
        Ok(Self {
            altitude_cutoff: add_parameter(context, 1, "ALT_CUTOFF", 0.0)?,
            distance_cutoff: add_parameter(context, 2, "DIST_CUTOFF", 0.0)?,
            enabled: parameter(context, "PLND_ENABLED")?,
            xy_distance_max: parameter(context, "PLND_XY_DIST_MAX")?,
            options: parameter(context, "PLND_OPTIONS")?,
        })
    }
}

struct Applet<'a, 'host> {
    parameters: Parameters<'a, 'host>,
    have_target: bool,
    precloiter_enabled: bool,
}

impl Applet<'_, '_> {
    fn update_target(&mut self, context: &Context<'_>, altitude_cutoff: f32) -> Result {
        if !retry(context, || context.precland().healthy())? {
            self.have_target = false;
            return Ok(());
        }
        let mut acquired = retry(context, || context.precland().target_acquired())?;
        if altitude_cutoff > 0.0
            && !retry(context, || {
                context.rangefinder().has_data_orient(RANGEFINDER_DOWN)
            })?
        {
            acquired = false;
        }
        if acquired != self.have_target {
            self.have_target = acquired;
            message(
                context,
                6,
                if acquired {
                    "PLND: Target Acquired"
                } else {
                    "PLND: Target Lost"
                },
            )?;
        }
        Ok(())
    }

    fn precision_landing_active(&self, context: &Context<'_>) -> Result<bool> {
        let mode = retry(context, || context.vehicle().get_mode())?;
        if mode == MODE_QLOITER {
            return Ok(self.precloiter_enabled);
        }
        Ok(retry(context, || context.quadplane().in_vtol_land_descent())? || mode == MODE_QLAND)
    }

    fn precloiter_check(&mut self, context: &Context<'_>) -> Result {
        let Some(position) = retry(context, || context.rc().get_aux_cached(AUX_PRECLOITER))? else {
            return Ok(());
        };
        let enabled = position == 2;
        if enabled != self.precloiter_enabled {
            self.precloiter_enabled = enabled;
            message(
                context,
                6,
                if enabled {
                    "PLND: PrecLoiter enabled"
                } else {
                    "PLND: PrecLoiter disabled"
                },
            )?;
        }
        Ok(())
    }

    fn update(&mut self, context: &Context<'_>) -> Result {
        if value(&self.parameters.enabled)? < 1.0 {
            return Ok(());
        }
        self.precloiter_check(context)?;

        // Fetch this before testing the descent state, matching the Lua applet's
        // ordering and avoiding a race with update_target_location().
        let Some(mut next_waypoint) = retry(context, || context.vehicle().get_target_location())?
        else {
            return Ok(());
        };
        if !self.precision_landing_active(context)? {
            return Ok(());
        }

        let altitude_cutoff = value(&self.parameters.altitude_cutoff)?;
        self.update_target(context, altitude_cutoff)?;
        if !self.have_target {
            return Ok(());
        }
        let Some(target) = retry(context, || context.precland().get_target_location())? else {
            return Ok(());
        };

        let rangefinder_distance = retry(context, || {
            context.rangefinder().distance_orient(RANGEFINDER_DOWN)
        })?;
        if altitude_cutoff > 0.0 && rangefinder_distance < altitude_cutoff {
            return Ok(());
        }

        let mut new_waypoint = next_waypoint.copy(context)?;
        new_waypoint.set_lat(context, target.lat())?;
        new_waypoint.set_lng(context, target.lng())?;
        let _ = retry(context, || {
            context.vehicle().update_target_location(&mut next_waypoint, &mut new_waypoint)
        })?;

        let mut vehicle_location = retry(context, || context.ahrs().get_location())?
            .ok_or(Error::Unavailable)?;
        let horizontal_distance = vehicle_location.get_distance(context, &mut new_waypoint)?;

        let mut target_velocity = retry(context, || context.precland().get_target_velocity())?;
        if let Some(velocity) = target_velocity.as_mut() {
            if (value(&self.parameters.options)? as i32 & 1) != 0 {
                let _ = retry(context, || context.vehicle().set_velocity_match(velocity))?;
            }
        }
        let velocity = target_velocity.unwrap_or(Vector2f { x: 0.0, y: 0.0 });

        context.logger().write(
            "PPLD",
            "Lat,Lon,Alt,HDist,RFND,VN,VE",
            Some(("DUmmmmm", "GG-----")),
            &[
                LogValue::LatLon(new_waypoint.lat()),
                LogValue::LatLon(new_waypoint.lng()),
                LogValue::Float(new_waypoint.alt() as f32),
                LogValue::Float(horizontal_distance),
                LogValue::Float(rangefinder_distance),
                LogValue::Float(velocity.x),
                LogValue::Float(velocity.y),
            ],
        )?;

        if value(&self.parameters.distance_cutoff)? > 0.0
            && horizontal_distance > value(&self.parameters.distance_cutoff)?
        {
            return Ok(());
        }
        if value(&self.parameters.xy_distance_max)? > 0.0
            && horizontal_distance > value(&self.parameters.xy_distance_max)?
        {
            let _ = retry(context, || context.vehicle().set_land_descent_rate(0.0))?;
        }
        Ok(())
    }
}

fn run(context: &Context<'_>) -> Result {
    let parameters = Parameters::new(context)?;
    if value(&parameters.enabled)? == 0.0 {
        message(context, 6, "PLND: Disabled")?;
        return Ok(());
    }
    let mut applet = Applet {
        parameters,
        have_target: false,
        precloiter_enabled: false,
    };
    message(context, 6, "PLND: Loaded")?;

    loop {
        if let Err(error) = applet.update(context) {
            if error == Error::Stopped {
                return Err(error);
            }
            // A contended firmware semaphore is expected under load. The Lua
            // binding waits for it; native bindings are bounded, so defer the
            // idempotent update to the next tick without alarming the pilot.
            if error == Error::Busy {
                context.sleep_ms(100)?;
                continue;
            }
            let text = Text::<96>::new(format_args!("Internal Error: {error:?}"))?;
            message(context, 0, text.as_str())?;
            context.sleep_ms(1_000)?;
        } else {
            context.sleep_ms(100)?;
        }
    }
}

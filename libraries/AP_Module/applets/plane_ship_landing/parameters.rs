// SPDX-License-Identifier: GPL-3.0-or-later

use ap_rust_sdk::{Context, Error, Parameter, Result};

const TABLE_KEY: u8 = 7;

pub fn retry<T>(context: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
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

fn bind<'a, 'host>(context: &'a Context<'host>, name: &str) -> Result<Parameter<'a, 'host>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Invalid)
}

fn add<'a, 'host>(
    context: &'a Context<'host>,
    index: u8,
    name: &str,
    full_name: &str,
    default: f32,
) -> Result<Parameter<'a, 'host>> {
    if !retry(context, || {
        context.param().add_param(TABLE_KEY, index, name, default)
    })? {
        return Err(Error::Host);
    }
    bind(context, full_name)
}

pub fn read(parameter: &Parameter<'_, '_>) -> Result<f32> {
    let value = parameter.get()?.ok_or(Error::Invalid)?;
    value.is_finite().then_some(value).ok_or(Error::Invalid)
}

pub fn save(parameter: &Parameter<'_, '_>, value: f32) -> Result {
    if parameter.set_and_save(value)? {
        Ok(())
    } else {
        Err(Error::Host)
    }
}

pub struct Parameters<'ctx, 'host> {
    pub enable: Parameter<'ctx, 'host>,
    pub land_angle: Parameter<'ctx, 'host>,
    pub auto_offset: Parameter<'ctx, 'host>,
    pub rcmap_throttle: Parameter<'ctx, 'host>,
    pub rtl_altitude: Parameter<'ctx, 'host>,
    pub q_rtl_alt: Parameter<'ctx, 'host>,
    pub airspeed_cruise: Parameter<'ctx, 'host>,
    pub tecs_land_airspeed: Parameter<'ctx, 'host>,
    pub q_transition_decel: Parameter<'ctx, 'host>,
    pub wp_loiter_radius: Parameter<'ctx, 'host>,
    pub rtl_radius: Parameter<'ctx, 'host>,
    pub foll_ofs_x: Parameter<'ctx, 'host>,
    pub foll_ofs_y: Parameter<'ctx, 'host>,
    pub foll_ofs_z: Parameter<'ctx, 'host>,
    pub foll_enable: Parameter<'ctx, 'host>,
    pub foll_ofs_type: Parameter<'ctx, 'host>,
    pub foll_alt_type: Parameter<'ctx, 'host>,
}

impl<'ctx, 'host> Parameters<'ctx, 'host> {
    pub fn new(context: &'ctx Context<'host>) -> Result<Self> {
        if !retry(context, || context.param().add_table(TABLE_KEY, "SHIP_", 3))? {
            return Err(Error::Host);
        }
        Ok(Self {
            enable: add(context, 1, "ENABLE", "SHIP_ENABLE", 0.0)?,
            land_angle: add(context, 2, "LAND_ANGLE", "SHIP_LAND_ANGLE", 0.0)?,
            auto_offset: add(context, 3, "AUTO_OFS", "SHIP_AUTO_OFS", 0.0)?,
            rcmap_throttle: bind(context, "RCMAP_THROTTLE")?,
            rtl_altitude: bind(context, "RTL_ALTITUDE")?,
            q_rtl_alt: bind(context, "Q_RTL_ALT")?,
            airspeed_cruise: bind(context, "AIRSPEED_CRUISE")?,
            tecs_land_airspeed: bind(context, "TECS_LAND_ARSPD")?,
            q_transition_decel: bind(context, "Q_TRANS_DECEL")?,
            wp_loiter_radius: bind(context, "WP_LOITER_RAD")?,
            rtl_radius: bind(context, "RTL_RADIUS")?,
            foll_ofs_x: bind(context, "FOLL_OFS_X")?,
            foll_ofs_y: bind(context, "FOLL_OFS_Y")?,
            foll_ofs_z: bind(context, "FOLL_OFS_Z")?,
            foll_enable: bind(context, "FOLL_ENABLE")?,
            foll_ofs_type: bind(context, "FOLL_OFS_TYPE")?,
            foll_alt_type: bind(context, "FOLL_ALT_TYPE")?,
        })
    }
}

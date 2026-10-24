// SPDX-License-Identifier: GPL-3.0-or-later

use ap_rust_sdk::{Context, Error, Parameter, Result};

const TABLE_KEY: u8 = 106;

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
    if !retry(context, || context.param().add_param(TABLE_KEY, index, name, default))? {
        return Err(Error::Host);
    }
    bind(context, full_name)
}

pub fn read(parameter: &Parameter<'_, '_>) -> Result<f32> {
    let value = parameter.get()?.ok_or(Error::Invalid)?;
    value.is_finite().then_some(value).ok_or(Error::Invalid)
}

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

pub struct Parameters<'ctx, 'host> {
    pub activation_function: Parameter<'ctx, 'host>,
    pub pitch_down_min: Parameter<'ctx, 'host>,
    pub pitch_forward_min: Parameter<'ctx, 'host>,
    pub quad_down_min: Parameter<'ctx, 'host>,
    pub quad_forward_min: Parameter<'ctx, 'host>,
    pub pitch_groundspeed_min: Parameter<'ctx, 'host>,
    pub pitch_timeout: Parameter<'ctx, 'host>,
    pub home_distance: Parameter<'ctx, 'host>,
    pub altitude_max: Parameter<'ctx, 'host>,
    pub groundspeed_max: Parameter<'ctx, 'host>,
    pub groundspeed_airbrake: Parameter<'ctx, 'host>,
    pub cmtc_height: Parameter<'ctx, 'host>,
    pub cmtc_enable: Parameter<'ctx, 'host>,
    pub update_rate: Parameter<'ctx, 'host>,
    pub cmtc_radius: Parameter<'ctx, 'host>,
    pub q_enable: Parameter<'ctx, 'host>,
    pub q_wvane_enable: Parameter<'ctx, 'host>,
    pub airspeed_min: Parameter<'ctx, 'host>,
    pub airspeed_cruise: Parameter<'ctx, 'host>,
    pub airspeed_max: Parameter<'ctx, 'host>,
    pub wp_loiter_radius: Parameter<'ctx, 'host>,
    pub terrain_enable: Parameter<'ctx, 'host>,
    pub terrain_spacing: Parameter<'ctx, 'host>,
    pub pitch_limit_max_deg: Parameter<'ctx, 'host>,
    pub tecs_climb_max: Parameter<'ctx, 'host>,
}

impl<'ctx, 'host> Parameters<'ctx, 'host> {
    pub fn new(context: &'ctx Context<'host>) -> Result<Self> {
        if !retry(context, || context.param().add_table(TABLE_KEY, "TA_", 19))? {
            return Err(Error::Host);
        }
        Ok(Self {
            activation_function: add(context, 1, "ACT_FN", "TA_ACT_FN", 305.0)?,
            pitch_down_min: add(context, 2, "PTCH_DWN_MIN", "TA_PTCH_DWN_MIN", 46.0)?,
            pitch_forward_min: add(context, 3, "PTCH_FWD_MIN", "TA_PTCH_FWD_MIN", 80.0)?,
            quad_down_min: add(context, 4, "QUAD_DWN_MIN", "TA_QUAD_DWN_MIN", 35.0)?,
            quad_forward_min: add(context, 5, "QUAD_FWD_MIN", "TA_QUAD_FWD_MIN", 20.0)?,
            pitch_groundspeed_min: add(context, 6, "PTCH_GSP_MIN", "TA_PTCH_GSP_MIN", 17.0)?,
            pitch_timeout: add(context, 7, "PTCH_TIMEOUT", "TA_PTCH_TIMEOUT", 2.0)?,
            home_distance: add(context, 8, "HOME_DIST", "TA_HOME_DIST", 100.0)?,
            altitude_max: add(context, 9, "ALT_MAX", "TA_ALT_MAX", 120.0)?,
            groundspeed_max: add(context, 10, "GSP_MAX", "TA_GSP_MAX", -1.0)?,
            groundspeed_airbrake: add(context, 11, "GSP_AIRBRAKE", "TA_GSP_AIRBRAKE", 0.0)?,
            cmtc_height: add(context, 12, "CMTC_HGT", "TA_CMTC_HGT", 50.0)?,
            cmtc_enable: add(context, 13, "CMTC_ENABLE", "TA_CMTC_ENABLE", 0.0)?,
            update_rate: add(context, 14, "UPDATE_RATE", "TA_UPDATE_RATE", 10.0)?,
            cmtc_radius: add(context, 15, "CMTC_RAD", "TA_CMTC_RAD", 0.0)?,
            q_enable: bind(context, "Q_ENABLE")?,
            q_wvane_enable: bind(context, "Q_WVANE_ENABLE")?,
            airspeed_min: bind(context, "AIRSPEED_MIN")?,
            airspeed_cruise: bind(context, "AIRSPEED_CRUISE")?,
            airspeed_max: bind(context, "AIRSPEED_MAX")?,
            wp_loiter_radius: bind(context, "WP_LOITER_RAD")?,
            terrain_enable: bind(context, "TERRAIN_ENABLE")?,
            terrain_spacing: bind(context, "TERRAIN_SPACING")?,
            pitch_limit_max_deg: bind(context, "PTCH_LIM_MAX_DEG")?,
            tecs_climb_max: bind(context, "TECS_CLMB_MAX")?,
        })
    }
}

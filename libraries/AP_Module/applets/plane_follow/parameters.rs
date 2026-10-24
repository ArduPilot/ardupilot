// SPDX-License-Identifier: GPL-3.0-or-later

use ap_rust_sdk::{Context, Error, Parameter, Result};

const TABLE_KEY: u8 = 123;

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

pub struct Parameters<'ctx, 'host> {
    pub foll_alt_type: Parameter<'ctx, 'host>,
    pub foll_sysid: Parameter<'ctx, 'host>,
    pub foll_ofs_y: Parameter<'ctx, 'host>,
    pub fail_mode: Parameter<'ctx, 'host>,
    pub exit_mode: Parameter<'ctx, 'host>,
    pub activation_function: Parameter<'ctx, 'host>,
    pub timeout: Parameter<'ctx, 'host>,
    pub overshoot_deg: Parameter<'ctx, 'host>,
    pub turn_deg: Parameter<'ctx, 'host>,
    pub distance_close: Parameter<'ctx, 'host>,
    pub wide_turns: Parameter<'ctx, 'host>,
    pub altitude_override: Parameter<'ctx, 'host>,
    pub distance_p: Parameter<'ctx, 'host>,
    pub distance_i: Parameter<'ctx, 'host>,
    pub distance_d: Parameter<'ctx, 'host>,
    pub velocity_p: Parameter<'ctx, 'host>,
    pub velocity_i: Parameter<'ctx, 'host>,
    pub velocity_d: Parameter<'ctx, 'host>,
    pub lookahead: Parameter<'ctx, 'host>,
    pub simulate_tlf_function: Parameter<'ctx, 'host>,
    pub cross_track_p: Parameter<'ctx, 'host>,
    pub cross_track_i: Parameter<'ctx, 'host>,
    pub cross_track_d: Parameter<'ctx, 'host>,
    pub cross_track_max: Parameter<'ctx, 'host>,
    pub cross_track_i_max: Parameter<'ctx, 'host>,
    pub refresh: Parameter<'ctx, 'host>,
    pub airspeed_min: Parameter<'ctx, 'host>,
    pub airspeed_max: Parameter<'ctx, 'host>,
    pub windspeed_max: Parameter<'ctx, 'host>,
}

impl<'ctx, 'host> Parameters<'ctx, 'host> {
    pub fn new(context: &'ctx Context<'host>) -> Result<Self> {
        if !retry(context, || context.param().add_table(TABLE_KEY, "FOLLP_", 30))? {
            return Err(Error::Host);
        }
        Ok(Self {
            foll_alt_type: bind(context, "FOLL_ALT_TYPE")?,
            foll_sysid: bind(context, "FOLL_SYSID")?,
            foll_ofs_y: bind(context, "FOLL_OFS_Y")?,
            fail_mode: add(context, 1, "FAIL_MODE", "FOLLP_FAIL_MODE", 11.0)?,
            exit_mode: add(context, 2, "EXIT_MODE", "FOLLP_EXIT_MODE", 12.0)?,
            activation_function: add(context, 3, "ACT_FN", "FOLLP_ACT_FN", 301.0)?,
            timeout: add(context, 4, "TIMEOUT", "FOLLP_TIMEOUT", 10.0)?,
            overshoot_deg: add(context, 5, "OVRSHT_DEG", "FOLLP_OVRSHT_DEG", 75.0)?,
            turn_deg: add(context, 6, "TURN_DEG", "FOLLP_TURN_DEG", 15.0)?,
            distance_close: add(context, 7, "DIST_CLOSE", "FOLLP_DIST_CLOSE", 75.0)?,
            wide_turns: add(context, 8, "WIDE_TURNS", "FOLLP_WIDE_TURNS", 1.0)?,
            altitude_override: add(context, 9, "ALT_OVR", "FOLLP_ALT_OVR", 0.0)?,
            distance_p: add(context, 10, "D_P", "FOLLP_D_P", 0.00025)?,
            distance_i: add(context, 11, "D_I", "FOLLP_D_I", 0.00025)?,
            distance_d: add(context, 12, "D_D", "FOLLP_D_D", 0.00013)?,
            velocity_p: add(context, 13, "V_P", "FOLLP_V_P", 0.00025)?,
            velocity_i: add(context, 14, "V_I", "FOLLP_V_I", 0.00025)?,
            velocity_d: add(context, 15, "V_D", "FOLLP_V_D", 0.0005)?,
            lookahead: add(context, 16, "LKAHD", "FOLLP_LKAHD", 3.0)?,
            simulate_tlf_function: add(context, 17, "SIM_TLF_FN", "FOLLP_SIM_TLF_FN", 302.0)?,
            cross_track_p: add(context, 20, "XT_P", "FOLLP_XT_P", 0.8)?,
            cross_track_i: add(context, 21, "XT_I", "FOLLP_XT_I", 0.01)?,
            cross_track_d: add(context, 22, "XT_D", "FOLLP_XT_D", 0.5)?,
            cross_track_max: add(context, 23, "XT_MAX", "FOLLP_XT_MAX", 45.0)?,
            cross_track_i_max: add(context, 24, "XT_I_MAX", "FOLLP_XT_I_MAX", 100.0)?,
            refresh: add(context, 25, "REFRESH", "FOLLP_REFRESH", 0.05)?,
            airspeed_min: bind(context, "AIRSPEED_MIN")?,
            airspeed_max: bind(context, "AIRSPEED_MAX")?,
            windspeed_max: bind(context, "AHRS_WIND_MAX")?,
        })
    }
}

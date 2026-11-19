// SPDX-License-Identifier: GPL-3.0-or-later

use ap_rust_sdk::{Context, Error, Parameter, Result};

ap_rust_sdk::param_table! {
    pub struct Table, enum P;
    key = 106, prefix = "TA_";
    size = 19;

    // @Param: TA_ACT_FN
    // @DisplayName: Activation Function for Terrain Avoidance
    // @Description: Setting an RC channel's _OPTION to this value will use it for Terrain Avoidance enable/disable
    // @Range: 300 307
    ActFn = 1, "ACT_FN", f32: 305.0;

    // @Param: TA_PTCH_DWN_MIN
    // @DisplayName: down distance minimum for Pitching
    // @Description: If the downward distance is less than this value then start Pitching up to gain altitude.
    // @Units: m
    PtchDwnMin = 2, "PTCH_DWN_MIN", f32: 46.0;

    // @Param: TA_PTCH_FWD_MIN
    // @DisplayName: forward distance minimum for Pitching
    // @Description: If the farwardward distance is less than this value then start Pitching up to gain altitude.
    // @Units: m
    PtchFwdMin = 3, "PTCH_FWD_MIN", f32: 80.0;

    // @Param: TA_QUAD_DWN_MIN
    // @DisplayName: Downward distance minimum Quading
    // @Description: If the downward distance is less than this value then start Quading up to gain altitude.
    // @Units: m
    QuadDwnMin = 4, "QUAD_DWN_MIN", f32: 35.0;

    // @Param: TA_QUAD_FWD_MIN
    // @DisplayName: minimum forward distance for Quading
    // @Description: If the farwardward distance is less than this value then start Quading up to gain altitude.
    // @Units: m
    QuadFwdMin = 5, "QUAD_FWD_MIN", f32: 20.0;

    // @Param: TA_PTCH_GSP_MIN
    // @DisplayName: minimum ground speed for Pitching
    // @Description: Minimum Groundspeed (not airspeed) to be flying for Pitching to be used.
    // @Units: m/s
    PtchGspMin = 6, "PTCH_GSP_MIN", f32: 17.0;

    // @Param: TA_PTCH_TIMEOUT
    // @DisplayName: timeout Pitching
    // @Description: Minimum down or forward distance must be triggered for more than this many seconds to start Pitching
    // @Units: s
    PtchTimeout = 7, "PTCH_TIMEOUT", f32: 2.0;

    // @Param: TA_HOME_DIST
    // @DisplayName: safe distance around home
    // @Description: Terrain avoidance will not be applied if the vehicle is less than this distance from home
    // @Units: m
    HomeDist = 8, "HOME_DIST", f32: 100.0;

    // @Param: TA_ALT_MAX
    // @DisplayName: ceiling for pitching/quading
    // @Description: This is a limit on how high the terrain avoidane will take the vehicle. It acts a failsafe to prevent vertical flyaways.
    // @Range: 20 1000
    // @Units: m
    AltMax = 9, "ALT_MAX", f32: 120.0;

    // @Param: TA_GSP_MAX
    // @DisplayName: Maximum Groundspeed
    // @Description: This is a limit on how fast in groundspeeed terrain avoidance will take the vehicle. This is to allow for reliable sensor readings. -1 for disabled.
    // @Range: 10 40
    // @Units: m/s
    GspMax = 10, "GSP_MAX", f32: -1.0;

    // @Param: TA_GSP_AIRBRAKE
    // @DisplayName: Groudspeed Airbrake limt
    // @Description: This is the limit for triggering airbrake to slow groundspeed as a difference between the airspeed and groundspeed. -1 for disabled.
    // @Range: -1 -10
    // @Units: m/s
    GspAirbrake = 11, "GSP_AIRBRAKE", f32: 0.0;

    // @Param: TA_CMTC_HGT
    // @DisplayName: CMTC Height
    // @Description: The minimum Height above terrain to maintain when following an AUTO mission or RTL. If zero(0) use TA_PTCH_DOW_MIN.
    // @Units: m
    CmtcHgt = 12, "CMTC_HGT", f32: 50.0;

    // @Param: TA_CMTC_ENABLE
    // @DisplayName: CMTC Enable
    // @Description: Whether to enable Can't Make That Climb while running Terrain Avoidance
    // @Range: 0 1
    CmtcEnable = 13, "CMTC_ENABLE", f32: 0.0;

    // @Param: TA_UPDATE_RATE
    // @DisplayName: Frequency to process avoidance
    // @Description: Avoidance processing rate
    // @Units: Hz
    UpdateRate = 14, "UPDATE_RATE", f32: 10.0;

    // @Param: TA_CMTC_RAD
    // @DisplayName: CMTC loiter radius
    // @Description: Use this radius for the loiter when trying to gain altitude. If not set or <=0 use WP_LOITER_RAD
    // @Units: m
    CmtcRad = 15, "CMTC_RAD", f32: 0.0;
}

fn bind<'a, 'host>(context: &'a Context<'host>, name: &str) -> Result<Parameter<'a, 'host>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Invalid)
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
        Table::register_definitions(context)?;
        Ok(Self {
            activation_function: Table::bind(context, P::ActFn)?,
            pitch_down_min: Table::bind(context, P::PtchDwnMin)?,
            pitch_forward_min: Table::bind(context, P::PtchFwdMin)?,
            quad_down_min: Table::bind(context, P::QuadDwnMin)?,
            quad_forward_min: Table::bind(context, P::QuadFwdMin)?,
            pitch_groundspeed_min: Table::bind(context, P::PtchGspMin)?,
            pitch_timeout: Table::bind(context, P::PtchTimeout)?,
            home_distance: Table::bind(context, P::HomeDist)?,
            altitude_max: Table::bind(context, P::AltMax)?,
            groundspeed_max: Table::bind(context, P::GspMax)?,
            groundspeed_airbrake: Table::bind(context, P::GspAirbrake)?,
            cmtc_height: Table::bind(context, P::CmtcHgt)?,
            cmtc_enable: Table::bind(context, P::CmtcEnable)?,
            update_rate: Table::bind(context, P::UpdateRate)?,
            cmtc_radius: Table::bind(context, P::CmtcRad)?,
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

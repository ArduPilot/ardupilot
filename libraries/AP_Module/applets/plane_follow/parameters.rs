// SPDX-License-Identifier: GPL-3.0-or-later

use ap_rust_sdk::{Context, Error, Parameter, Result};

ap_rust_sdk::param_table! {
    pub struct Table, enum P;
    key = 123, prefix = "FOLLP_";
    size = 30;

    // @Param: FOLLP_FAIL_MODE
    // @DisplayName: Plane Follow lost target mode
    // @Description: Mode to switch to if the target is lost (no signal or > FOLL_DIST_MAX).
    // @User: Standard
    FailMode = 1, "FAIL_MODE", f32: 11.0;

    // @Param: FOLLP_EXIT_MODE
    // @DisplayName: Plane Follow exit mode
    // @Description: Mode to switch to when follow mode is exited normally
    // @User: Standard
    ExitMode = 2, "EXIT_MODE", f32: 12.0;

    // @Param: FOLLP_ACT_FN
    // @DisplayName: Plane Follow Scripting ActivationFunction
    // @Description: Setting an RC channel's _OPTION to this value will use it for Plane Follow enable/disable
    // @Range: 300 307
    ActFn = 3, "ACT_FN", f32: 301.0;

    // @Param: FOLLP_TIMEOUT
    // @DisplayName: Plane Follow Telemetry Timeout
    // @Description: How long to try reaquire a target if telemetry from the lead vehicle is lost.
    // @Range: 0 30
    // @Units: s
    Timeout = 4, "TIMEOUT", f32: 10.0;

    // @Param: FOLLP_OVRSHT_DEG
    // @DisplayName: Plane Follow Scripting Overshoot Angle
    // @Description: If the target is greater than this many degrees left or right, assume an overshoot
    // @Range: 0 180
    // @Units: deg
    OvrshtDeg = 5, "OVRSHT_DEG", f32: 75.0;

    // @Param: FOLLP_TURN_DEG
    // @DisplayName: Plane Follow Scripting Turn Angle
    // @Description: If the target is greater than this many degrees left or right, assume it's turning
    // @Range: 0 180
    // @Units: deg
    TurnDeg = 6, "TURN_DEG", f32: 15.0;

    // @Param: FOLLP_DIST_CLOSE
    // @DisplayName: Plane Follow Scripting Close Distance
    // @Description: When closer than this distance assume we track by heading
    // @Range: 0 100
    // @Units: m
    DistClose = 7, "DIST_CLOSE", f32: 75.0;

    // @Param: FOLLP_WIDE_TURNS
    // @DisplayName: Plane Follow Scripting Wide Turns
    // @Description: Use wide turns when following a turning target. Alternative is "cutting the corner"
    // @Range: 0 1
    WideTurns = 8, "WIDE_TURNS", f32: 1.0;

    // @Param: FOLLP_ALT_OVR
    // @DisplayName: Plane Follow Scripting Altitude Override
    // @Description: When non zero, this altitude value (in FOLL_ALT_TYPE frame) overrides the value sent by the target vehicle
    // @Range: 0 1000
    // @Units: m
    AltOvr = 9, "ALT_OVR", f32: 0.0;

    // @Param: FOLLP_D_P
    // @DisplayName: Plane Follow Scripting distance P gain
    // @Description: P gain for the speed PID controller distance component
    // @Range: 0 1
    DP = 10, "D_P", f32: 0.00025;

    // @Param: FOLLP_D_I
    // @DisplayName: Plane Follow Scripting distance I gain
    // @Description: I gain for the speed PID  distance component
    // @Range: 0 1
    DI = 11, "D_I", f32: 0.00025;

    // @Param: FOLLP_D_D
    // @DisplayName: Plane Follow Scripting distance D gain
    // @Description: D gain for the speed PID controller distance component
    // @Range: 0 1
    DD = 12, "D_D", f32: 0.00013;

    // @Param: FOLLP_V_P
    // @DisplayName: Plane Follow Scripting speed P gain
    // @Description: P gain for the speed PID controller velocity component
    // @Range: 0 1
    VP = 13, "V_P", f32: 0.00025;

    // @Param: FOLLP_V_I
    // @DisplayName: Plane Follow Scripting speed I gain
    // @Description: I gain for the speed PID controller velocity component
    // @Range: 0 1
    VI = 14, "V_I", f32: 0.00025;

    // @Param: FOLLP_V_D
    // @DisplayName: Plane Follow Scripting speed D gain
    // @Description: D gain for the speed PID controller velocity component
    // @Range: 0 1
    VD = 15, "V_D", f32: 0.0005;

    // @Param: FOLLP_LKAHD
    // @DisplayName: Plane Follow Lookahead seconds
    // @Description: Time to "lookahead" when calculating distance errors
    // @Units: s
    Lkahd = 16, "LKAHD", f32: 3.0;

    // @Param: FOLLP_SIM_TLF_FN
    // @DisplayName: Plane Follow Simulate Telemetry fail function
    // @Description: Set this switch to simulate losing telemetry from the other vehicle
    // @Range: 300 307
    SimTlfFn = 17, "SIM_TLF_FN", f32: 302.0;

    // @Param: FOLLP_XT_P
    // @DisplayName: Plane Follow crosstrack PID controller P term
    // @Description: P term for the crosstrack/heading PID controller
    // @Range: 0 1
    XtP = 20, "XT_P", f32: 0.8;

    // @Param: FOLLP_XT_I
    // @DisplayName: Plane Follow crosstrack PID controller I term
    // @Description: I term for the crosstrack/heading PID controller
    // @Range: 0 1
    XtI = 21, "XT_I", f32: 0.01;

    // @Param: FOLLP_XT_D
    // @DisplayName: Plane Follow crosstrack PID controller D term
    // @Description: D term for the crosstrack/heading PID controller
    // @Range: 0 1
    XtD = 22, "XT_D", f32: 0.5;

    // @Param: FOLLP_XT_MAX
    // @DisplayName: Plane Follow crosstrack PID controller maximum correction
    // @Description: maximum correction retured by the crosstrack/heading PID controller
    // @Range: 0 1
    // @Units: deg
    XtMax = 23, "XT_MAX", f32: 45.0;

    // @Param: FOLLP_XT_I_MAX
    // @DisplayName: Plane Follow crosstrack PID controller maximum integral
    // @Description: maximum I applied the crosstrack/heading PID controller
    // @Range: 0 100
    // @Units: ms
    XtIMax = 24, "XT_I_MAX", f32: 100.0;

    // @Param: FOLLP_REFRESH
    // @DisplayName: Plane Follow refresh rate
    // @Description: refresh rate for Plane Follow updates
    // @Range: 0 0.2
    // @Units: s
    Refresh = 25, "REFRESH", f32: 0.05;
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

fn bind<'a, 'host>(context: &'a Context<'host>, name: &str) -> Result<Parameter<'a, 'host>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Invalid)
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
        Table::register_definitions(context)?;
        Ok(Self {
            foll_alt_type: bind(context, "FOLL_ALT_TYPE")?,
            foll_sysid: bind(context, "FOLL_SYSID")?,
            foll_ofs_y: bind(context, "FOLL_OFS_Y")?,
            fail_mode: Table::bind(context, P::FailMode)?,
            exit_mode: Table::bind(context, P::ExitMode)?,
            activation_function: Table::bind(context, P::ActFn)?,
            timeout: Table::bind(context, P::Timeout)?,
            overshoot_deg: Table::bind(context, P::OvrshtDeg)?,
            turn_deg: Table::bind(context, P::TurnDeg)?,
            distance_close: Table::bind(context, P::DistClose)?,
            wide_turns: Table::bind(context, P::WideTurns)?,
            altitude_override: Table::bind(context, P::AltOvr)?,
            distance_p: Table::bind(context, P::DP)?,
            distance_i: Table::bind(context, P::DI)?,
            distance_d: Table::bind(context, P::DD)?,
            velocity_p: Table::bind(context, P::VP)?,
            velocity_i: Table::bind(context, P::VI)?,
            velocity_d: Table::bind(context, P::VD)?,
            lookahead: Table::bind(context, P::Lkahd)?,
            simulate_tlf_function: Table::bind(context, P::SimTlfFn)?,
            cross_track_p: Table::bind(context, P::XtP)?,
            cross_track_i: Table::bind(context, P::XtI)?,
            cross_track_d: Table::bind(context, P::XtD)?,
            cross_track_max: Table::bind(context, P::XtMax)?,
            cross_track_i_max: Table::bind(context, P::XtIMax)?,
            refresh: Table::bind(context, P::Refresh)?,
            airspeed_min: bind(context, "AIRSPEED_MIN")?,
            airspeed_max: bind(context, "AIRSPEED_MAX")?,
            windspeed_max: bind(context, "AHRS_WIND_MAX")?,
        })
    }
}

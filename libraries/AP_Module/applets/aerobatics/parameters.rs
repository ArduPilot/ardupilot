// SPDX-License-Identifier: GPL-3.0-or-later
//! Shared Lua parameter layout, with a native-module language selector.

use super::{
    controller::Configuration,
    math::constrain,
    support::retry,
};
use ap_rust_sdk::{Context, Error, Parameter, Result};

ap_rust_sdk::param_table! {
    pub struct AeroTable, enum AeroParam;
    key = 70, prefix = "AEROM_";
    size = 40;

    // @Param: AEROM_ANG_ACCEL
    // @DisplayName: Angular acceleration limit
    // @Description: Maximum angular acceleration in maneuvers
    // @Units: deg/s/s
    AngAccel = 1, "ANG_ACCEL", f32: 6000.0;

    // @Param: AEROM_ANG_TC
    // @DisplayName: Roll control filtertime constant
    // @Description: This is the time over which we filter the desired roll to smooth it
    // @Units: s
    AngTc = 2, "ANG_TC", f32: 0.1;

    // @Param: AEROM_THR_PIT_FF
    // @DisplayName: Throttle feed forward from pitch
    // @Description: This controls how much extra throttle to add based on pitch ange. The value is for 90 degrees and is applied in proportion to pitch
    // @Units: %
    ThrPitFf = 4, "THR_PIT_FF", f32: 80.0;

    // @Param: AEROM_SPD_P
    // @DisplayName: P gain for speed controller
    // @Description: This controls how rapidly the throttle is raised to compensate for a speed error
    // @Units: %
    SpdP = 5, "SPD_P", f32: 5.0;

    // @Param: AEROM_SPD_I
    // @DisplayName: I gain for speed controller
    // @Description: This controls how rapidly the throttle is raised to compensate for a speed error
    // @Units: %
    SpdI = 6, "SPD_I", f32: 25.0;

    // @Param: AEROM_ROL_COR_TC
    // @DisplayName: Roll control time constant
    // @Description: This is the time constant for correcting roll errors. A smaller value leads to faster roll corrections
    // @Units: s
    RolCorTc = 8, "ROL_COR_TC", f32: 0.25;

    // @Param: AEROM_TIME_COR_P
    // @DisplayName: Time constant for correction of our distance along the path
    // @Description: This is the time constant for correcting path position errors
    // @Units: s
    TimeCorP = 11, "TIME_COR_P", f32: 1.0;

    // @Param: AEROM_ERR_COR_P
    // @DisplayName: P gain for path error corrections
    // @Description: This controls how rapidly we correct back onto the desired path
    ErrCorP = 12, "ERR_COR_P", f32: 2.0;

    // @Param: AEROM_ERR_COR_D
    // @DisplayName: D gain for path error corrections
    // @Description: This controls how rapidly we correct back onto the desired path
    ErrCorD = 13, "ERR_COR_D", f32: 2.8;

    // @Param: AEROM_ENTRY_RATE
    // @DisplayName: The roll rate to use when entering a roll maneuver
    // @Description: This controls how rapidly we roll into a new orientation
    // @Units: deg/s
    EntryRate = 14, "ENTRY_RATE", f32: 60.0;

    // @Param: AEROM_THR_LKAHD
    // @DisplayName: The lookahead for throttle control
    // @Description: This controls how far ahead we look in time along the path for the target throttle
    // @Units: s
    ThrLkahd = 15, "THR_LKAHD", f32: 1.0;

    // @Param: AEROM_DEBUG
    // @DisplayName: Debug control
    // @Description: This controls the printing of extra debug information on paths
    Debug = 16, "DEBUG", f32: 0.0;

    // @Param: AEROM_THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: Lowest throttle used during maneuvers
    // @Units: %
    ThrMin = 17, "THR_MIN", f32: 0.0;

    // @Param: AEROM_THR_BOOST
    // @DisplayName: Throttle boost
    // @Description: This is the extra throttle added in schedule elements marked as needing a throttle boost
    // @Units: %
    ThrBoost = 18, "THR_BOOST", f32: 50.0;

    // @Param: AEROM_YAW_ACCEL
    // @DisplayName: Yaw acceleration
    // @Description: This is maximum yaw acceleration to use
    // @Units: deg/s/s
    YawAccel = 19, "YAW_ACCEL", f32: 1500.0;

    // @Param: AEROM_LKAHD
    // @DisplayName: Lookahead
    // @Description: This is how much time to look ahead in the path for calculating path rates
    // @Units: s
    Lkahd = 20, "LKAHD", f32: 0.5;

    // @Param: AEROM_PATH_SCALE
    // @DisplayName: Path Scale
    // @Description: Scale factor for Path/Box size. 0.5 would half the distances in maneuvers. Radii are unaffected.
    // @Range: 0.1 100
    PathScale = 21, "PATH_SCALE", f32: 1.0;

    // @Param: AEROM_BOX_WIDTH
    // @DisplayName: Box Width
    // @Description: Length of aerobatic "box"
    // @Units: m
    BoxWidth = 22, "BOX_WIDTH", f32: 400.0;

    // @Param: AEROM_STALL_THR
    // @DisplayName: Stall turn throttle
    // @Description: Amount of throttle to reduce to for a stall turn
    // @Units: %
    StallThr = 23, "STALL_THR", f32: 40.0;

    // @Param: AEROM_STALL_PIT
    // @DisplayName: Stall turn pitch threshold
    // @Description: Pitch threashold for moving to final stage of stall turn
    // @Units: deg
    StallPit = 24, "STALL_PIT", f32: -20.0;

    // @Param: AEROM_KE_RUDD
    // @DisplayName: KnifeEdge Rudder
    // @Description: Percent of rudder normally uses to sustain knife-edge at trick speed
    // @Units: %
    KeRudd = 26, "KE_RUDD", f32: 25.0;

    // @Param: AEROM_KE_RUDD_LK
    // @DisplayName: KnifeEdge Rudder lookahead
    // @Description: Time to look ahead in the path to calculate rudder correction for bank angle
    // @Units: s
    KeRuddLk = 27, "KE_RUDD_LK", f32: 0.25;

    // @Param: AEROM_ALT_ABORT
    // @DisplayName: Altitude Abort
    // @Description: Maximum allowable loss in altitude during a trick or sequence from its starting altitude.
    // @Units: m
    AltAbort = 28, "ALT_ABORT", f32: 15.0;

    // @Param: AEROM_TS_P
    // @DisplayName: Timesync P gain
    // @Description: This controls how rapidly two aircraft are brought back into time sync
    TsP = 29, "TS_P", f32: 0.33;

    // @Param: AEROM_TS_I
    // @DisplayName: Timesync I gain
    // @Description: This controls how rapidly two aircraft are brought back into time sync
    TsI = 30, "TS_I", f32: 0.33;

    // @Param: AEROM_TS_SPDMAX
    // @DisplayName: Timesync speed max
    // @Description: This sets the maximum speed adjustment for time sync between aircraft
    // @Units: m/s
    TsSpdmax = 31, "TS_SPDMAX", f32: 0.0;

    // @Param: AEROM_TS_RATE
    // @DisplayName: Timesync rate of send of NAMED_VALUE_FLOAT data
    // @Description: This sets the rate we send data for time sync between aircraft
    // @Units: Hz
    TsRate = 32, "TS_RATE", f32: 4.0;

    // @Param: AEROM_MIS_ANGLE
    // @DisplayName: Mission angle
    // @Description: When set to a non-zero value, this is the assumed direction of the mission. Otherwise the waypoint angle is used
    // @Units: deg
    MisAngle = 33, "MIS_ANGLE", f32: 0.0;

    // @Param: AEROM_OPTIONS
    // @DisplayName: Aerobatic options
    // @Description: Options to control aerobatic behavior
    // @Bitmask: 0:UseRTLOnAbort, 1:AddAtToMessages, 2:DualAircraftSynchronised
    Options = 34, "OPTIONS", f32: 0.0;

    // @Param: AEROM_LANG
    // @DisplayName: Applet language
    // @Description: Select the native aerobatics implementation. Other values leave all native implementations idle.
    // @Values: 0:Rust,1:Ada,2:Fortran
    // @User: Advanced
    Lang = 35, "LANG", f32: 0.0;
}

ap_rust_sdk::param_table! {
    pub struct TrickTable, enum TrickParam;
    key = 71, prefix = "TRIK";
    size = 63;

    // @Param: TRIK_ENABLE
    // @DisplayName: Tricks on Switch Enable
    // @Description: Enables Tricks on Switch. TRIK params hidden until enabled
    Enable = 1, "_ENABLE", f32: 0.0;

    // @Param: TRIK_SEL_FN
    // @DisplayName: Trik Selection Scripting Function
    // @Description: Setting an RC channel's _OPTION to this value will use it for trick selection
    // @Range: 301 307
    SelFn = 2, "_SEL_FN", f32: 301.0;

    // @Param: TRIK_ACT_FN
    // @DisplayName: Trik Action Scripting Function
    // @Description: Setting an RC channel's _OPTION to this value will use it for trick action (abort,announce,execute)
    // @Range: 301 307
    ActFn = 3, "_ACT_FN", f32: 300.0;

    // @Param: TRIK_COUNT
    // @DisplayName: Trik Count
    // @Description: Number of tricks which can be selected over the range of the trik selection RC channel
    // @Range: 1 11
    Count = 4, "_COUNT", f32: 3.0;

    // @Param: TRIK1_ID
    // @DisplayName: Trick 1 identifier
    // @Description: Maneuver identifier for trick 1. Set to -1 to disable.
    // @User: Advanced
    Circuit1Id = 5, "1_ID", f32: -1.0;

    // @Param: TRIK1_ARG1
    // @DisplayName: Trick 1 argument 1
    // @Description: Argument 1 for trick 1; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit1Arg1 = 6, "1_ARG1", f32: 30.0;

    // @Param: TRIK1_ARG2
    // @DisplayName: Trick 1 argument 2
    // @Description: Argument 2 for trick 1; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit1Arg2 = 7, "1_ARG2", f32: 0.0;

    // @Param: TRIK1_ARG3
    // @DisplayName: Trick 1 argument 3
    // @Description: Argument 3 for trick 1; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit1Arg3 = 8, "1_ARG3", f32: 0.0;

    // @Param: TRIK1_ARG4
    // @DisplayName: Trick 1 argument 4
    // @Description: Argument 4 for trick 1; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit1Arg4 = 9, "1_ARG4", f32: 0.0;

    // @Param: TRIK2_ID
    // @DisplayName: Trick 2 identifier
    // @Description: Maneuver identifier for trick 2. Set to -1 to disable.
    // @User: Advanced
    Circuit2Id = 10, "2_ID", f32: -1.0;

    // @Param: TRIK2_ARG1
    // @DisplayName: Trick 2 argument 1
    // @Description: Argument 1 for trick 2; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit2Arg1 = 11, "2_ARG1", f32: 30.0;

    // @Param: TRIK2_ARG2
    // @DisplayName: Trick 2 argument 2
    // @Description: Argument 2 for trick 2; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit2Arg2 = 12, "2_ARG2", f32: 0.0;

    // @Param: TRIK2_ARG3
    // @DisplayName: Trick 2 argument 3
    // @Description: Argument 3 for trick 2; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit2Arg3 = 13, "2_ARG3", f32: 0.0;

    // @Param: TRIK2_ARG4
    // @DisplayName: Trick 2 argument 4
    // @Description: Argument 4 for trick 2; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit2Arg4 = 14, "2_ARG4", f32: 0.0;

    // @Param: TRIK3_ID
    // @DisplayName: Trick 3 identifier
    // @Description: Maneuver identifier for trick 3. Set to -1 to disable.
    // @User: Advanced
    Circuit3Id = 15, "3_ID", f32: -1.0;

    // @Param: TRIK3_ARG1
    // @DisplayName: Trick 3 argument 1
    // @Description: Argument 1 for trick 3; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit3Arg1 = 16, "3_ARG1", f32: 30.0;

    // @Param: TRIK3_ARG2
    // @DisplayName: Trick 3 argument 2
    // @Description: Argument 2 for trick 3; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit3Arg2 = 17, "3_ARG2", f32: 0.0;

    // @Param: TRIK3_ARG3
    // @DisplayName: Trick 3 argument 3
    // @Description: Argument 3 for trick 3; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit3Arg3 = 18, "3_ARG3", f32: 0.0;

    // @Param: TRIK3_ARG4
    // @DisplayName: Trick 3 argument 4
    // @Description: Argument 4 for trick 3; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit3Arg4 = 19, "3_ARG4", f32: 0.0;

    // @Param: TRIK4_ID
    // @DisplayName: Trick 4 identifier
    // @Description: Maneuver identifier for trick 4. Set to -1 to disable.
    // @User: Advanced
    Circuit4Id = 20, "4_ID", f32: -1.0;

    // @Param: TRIK4_ARG1
    // @DisplayName: Trick 4 argument 1
    // @Description: Argument 1 for trick 4; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit4Arg1 = 21, "4_ARG1", f32: 30.0;

    // @Param: TRIK4_ARG2
    // @DisplayName: Trick 4 argument 2
    // @Description: Argument 2 for trick 4; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit4Arg2 = 22, "4_ARG2", f32: 0.0;

    // @Param: TRIK4_ARG3
    // @DisplayName: Trick 4 argument 3
    // @Description: Argument 3 for trick 4; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit4Arg3 = 23, "4_ARG3", f32: 0.0;

    // @Param: TRIK4_ARG4
    // @DisplayName: Trick 4 argument 4
    // @Description: Argument 4 for trick 4; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit4Arg4 = 24, "4_ARG4", f32: 0.0;

    // @Param: TRIK5_ID
    // @DisplayName: Trick 5 identifier
    // @Description: Maneuver identifier for trick 5. Set to -1 to disable.
    // @User: Advanced
    Circuit5Id = 25, "5_ID", f32: -1.0;

    // @Param: TRIK5_ARG1
    // @DisplayName: Trick 5 argument 1
    // @Description: Argument 1 for trick 5; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit5Arg1 = 26, "5_ARG1", f32: 30.0;

    // @Param: TRIK5_ARG2
    // @DisplayName: Trick 5 argument 2
    // @Description: Argument 2 for trick 5; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit5Arg2 = 27, "5_ARG2", f32: 0.0;

    // @Param: TRIK5_ARG3
    // @DisplayName: Trick 5 argument 3
    // @Description: Argument 3 for trick 5; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit5Arg3 = 28, "5_ARG3", f32: 0.0;

    // @Param: TRIK5_ARG4
    // @DisplayName: Trick 5 argument 4
    // @Description: Argument 4 for trick 5; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit5Arg4 = 29, "5_ARG4", f32: 0.0;

    // @Param: TRIK6_ID
    // @DisplayName: Trick 6 identifier
    // @Description: Maneuver identifier for trick 6. Set to -1 to disable.
    // @User: Advanced
    Circuit6Id = 30, "6_ID", f32: -1.0;

    // @Param: TRIK6_ARG1
    // @DisplayName: Trick 6 argument 1
    // @Description: Argument 1 for trick 6; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit6Arg1 = 31, "6_ARG1", f32: 30.0;

    // @Param: TRIK6_ARG2
    // @DisplayName: Trick 6 argument 2
    // @Description: Argument 2 for trick 6; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit6Arg2 = 32, "6_ARG2", f32: 0.0;

    // @Param: TRIK6_ARG3
    // @DisplayName: Trick 6 argument 3
    // @Description: Argument 3 for trick 6; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit6Arg3 = 33, "6_ARG3", f32: 0.0;

    // @Param: TRIK6_ARG4
    // @DisplayName: Trick 6 argument 4
    // @Description: Argument 4 for trick 6; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit6Arg4 = 34, "6_ARG4", f32: 0.0;

    // @Param: TRIK7_ID
    // @DisplayName: Trick 7 identifier
    // @Description: Maneuver identifier for trick 7. Set to -1 to disable.
    // @User: Advanced
    Circuit7Id = 35, "7_ID", f32: -1.0;

    // @Param: TRIK7_ARG1
    // @DisplayName: Trick 7 argument 1
    // @Description: Argument 1 for trick 7; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit7Arg1 = 36, "7_ARG1", f32: 30.0;

    // @Param: TRIK7_ARG2
    // @DisplayName: Trick 7 argument 2
    // @Description: Argument 2 for trick 7; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit7Arg2 = 37, "7_ARG2", f32: 0.0;

    // @Param: TRIK7_ARG3
    // @DisplayName: Trick 7 argument 3
    // @Description: Argument 3 for trick 7; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit7Arg3 = 38, "7_ARG3", f32: 0.0;

    // @Param: TRIK7_ARG4
    // @DisplayName: Trick 7 argument 4
    // @Description: Argument 4 for trick 7; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit7Arg4 = 39, "7_ARG4", f32: 0.0;

    // @Param: TRIK8_ID
    // @DisplayName: Trick 8 identifier
    // @Description: Maneuver identifier for trick 8. Set to -1 to disable.
    // @User: Advanced
    Circuit8Id = 40, "8_ID", f32: -1.0;

    // @Param: TRIK8_ARG1
    // @DisplayName: Trick 8 argument 1
    // @Description: Argument 1 for trick 8; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit8Arg1 = 41, "8_ARG1", f32: 30.0;

    // @Param: TRIK8_ARG2
    // @DisplayName: Trick 8 argument 2
    // @Description: Argument 2 for trick 8; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit8Arg2 = 42, "8_ARG2", f32: 0.0;

    // @Param: TRIK8_ARG3
    // @DisplayName: Trick 8 argument 3
    // @Description: Argument 3 for trick 8; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit8Arg3 = 43, "8_ARG3", f32: 0.0;

    // @Param: TRIK8_ARG4
    // @DisplayName: Trick 8 argument 4
    // @Description: Argument 4 for trick 8; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit8Arg4 = 44, "8_ARG4", f32: 0.0;

    // @Param: TRIK9_ID
    // @DisplayName: Trick 9 identifier
    // @Description: Maneuver identifier for trick 9. Set to -1 to disable.
    // @User: Advanced
    Circuit9Id = 45, "9_ID", f32: -1.0;

    // @Param: TRIK9_ARG1
    // @DisplayName: Trick 9 argument 1
    // @Description: Argument 1 for trick 9; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit9Arg1 = 46, "9_ARG1", f32: 30.0;

    // @Param: TRIK9_ARG2
    // @DisplayName: Trick 9 argument 2
    // @Description: Argument 2 for trick 9; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit9Arg2 = 47, "9_ARG2", f32: 0.0;

    // @Param: TRIK9_ARG3
    // @DisplayName: Trick 9 argument 3
    // @Description: Argument 3 for trick 9; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit9Arg3 = 48, "9_ARG3", f32: 0.0;

    // @Param: TRIK9_ARG4
    // @DisplayName: Trick 9 argument 4
    // @Description: Argument 4 for trick 9; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit9Arg4 = 49, "9_ARG4", f32: 0.0;

    // @Param: TRIK10_ID
    // @DisplayName: Trick 10 identifier
    // @Description: Maneuver identifier for trick 10. Set to -1 to disable.
    // @User: Advanced
    Circuit10Id = 50, "10_ID", f32: -1.0;

    // @Param: TRIK10_ARG1
    // @DisplayName: Trick 10 argument 1
    // @Description: Argument 1 for trick 10; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit10Arg1 = 51, "10_ARG1", f32: 30.0;

    // @Param: TRIK10_ARG2
    // @DisplayName: Trick 10 argument 2
    // @Description: Argument 2 for trick 10; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit10Arg2 = 52, "10_ARG2", f32: 0.0;

    // @Param: TRIK10_ARG3
    // @DisplayName: Trick 10 argument 3
    // @Description: Argument 3 for trick 10; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit10Arg3 = 53, "10_ARG3", f32: 0.0;

    // @Param: TRIK10_ARG4
    // @DisplayName: Trick 10 argument 4
    // @Description: Argument 4 for trick 10; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit10Arg4 = 54, "10_ARG4", f32: 0.0;

    // @Param: TRIK11_ID
    // @DisplayName: Trick 11 identifier
    // @Description: Maneuver identifier for trick 11. Set to -1 to disable.
    // @User: Advanced
    Circuit11Id = 55, "11_ID", f32: -1.0;

    // @Param: TRIK11_ARG1
    // @DisplayName: Trick 11 argument 1
    // @Description: Argument 1 for trick 11; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit11Arg1 = 56, "11_ARG1", f32: 30.0;

    // @Param: TRIK11_ARG2
    // @DisplayName: Trick 11 argument 2
    // @Description: Argument 2 for trick 11; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit11Arg2 = 57, "11_ARG2", f32: 0.0;

    // @Param: TRIK11_ARG3
    // @DisplayName: Trick 11 argument 3
    // @Description: Argument 3 for trick 11; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit11Arg3 = 58, "11_ARG3", f32: 0.0;

    // @Param: TRIK11_ARG4
    // @DisplayName: Trick 11 argument 4
    // @Description: Argument 4 for trick 11; meaning depends on its maneuver identifier.
    // @User: Advanced
    Circuit11Arg4 = 59, "11_ARG4", f32: 0.0;
}

pub struct Parameters<'a, 'h> {
    aerom: [Option<Parameter<'a, 'h>>; 41],
    pub trik: [Option<Parameter<'a, 'h>>; 64],
    airspeed_min: Parameter<'a, 'h>,
    scaling_speed: Parameter<'a, 'h>,
    airspeed_cruise: Parameter<'a, 'h>,
    // Lua captures speed PI gains at script startup, not on each update.
    spd_p: f32,
    spd_i: f32,
    pub trick_count: usize,
}

pub fn read(p: &Parameter<'_, '_>) -> Result<f32> {
    let value = p.get()?.ok_or(Error::Invalid)?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(Error::Invalid)
    }
}

fn bind<'a, 'h>(c: &'a Context<'h>, name: &str) -> Result<Parameter<'a, 'h>> {
    retry(c, || c.parameter(name))?.ok_or(Error::Invalid)
}

impl<'a, 'h> Parameters<'a, 'h> {
    pub fn new(c: &'a Context<'h>) -> Result<Self> {
        AeroTable::register_definitions(c)?;
        TrickTable::register_table(c)?;
        let mut aerom = core::array::from_fn(|_| None);
        for d in AeroTable::definitions() {
            aerom[d.index as usize] = Some(d.bind(c, AeroTable::PREFIX)?);
        }
        let mut trik = core::array::from_fn(|_| None);
        let enable = TrickTable::definition(TrickParam::Enable);
        enable.register(c, TrickTable::KEY)?;
        trik[1] = Some(enable.bind(c, TrickTable::PREFIX)?);
        let mut result = Self {
            spd_p: read(aerom[5].as_ref().unwrap())?,
            spd_i: read(aerom[6].as_ref().unwrap())?,
            aerom,
            trik,
            airspeed_min: bind(c, "AIRSPEED_MIN")?,
            scaling_speed: bind(c, "SCALING_SPEED")?,
            airspeed_cruise: bind(c, "AIRSPEED_CRUISE")?,
            trick_count: 0,
        };
        let accel = result.a(1)?;
        if accel > 0.0 && accel < 100.0 {
            result.aerom[1].as_ref().unwrap().set_and_save(3000.0)?;
        }
        if result.a(2)? > 1.0 {
            result.aerom[2].as_ref().unwrap().set_and_save(0.2)?;
        }
        if result.t(1)? > 0.0 {
            for d in &TrickTable::definitions()[1..4] {
                d.register(c, TrickTable::KEY)?;
                result.trik[d.index as usize] = Some(d.bind(c, TrickTable::PREFIX)?);
            }
            result.trick_count = c.libm().floorf(constrain(result.t(4)?, 1.0, 11.0))? as usize;
            for d in &TrickTable::definitions()[4..4 + 5 * result.trick_count] {
                d.register(c, TrickTable::KEY)?;
                result.trik[d.index as usize] = Some(d.bind(c, TrickTable::PREFIX)?);
            }
        }
        Ok(result)
    }

    pub fn a(&self, index: usize) -> Result<f32> {
        read(
            self.aerom
                .get(index)
                .and_then(Option::as_ref)
                .ok_or(Error::Invalid)?,
        )
    }
    pub fn t(&self, index: usize) -> Result<f32> {
        read(
            self.trik
                .get(index)
                .and_then(Option::as_ref)
                .ok_or(Error::Invalid)?,
        )
    }
    pub fn configuration(&self) -> Result<Configuration> {
        Ok(Configuration {
            ang_accel: self.a(1)?,
            ang_tc: self.a(2)?,
            thr_pit_ff: self.a(4)?,
            spd_p: self.spd_p,
            spd_i: self.spd_i,
            roll_corr_tc: self.a(8)?,
            time_corr_p: self.a(11)?,
            err_corr_p: self.a(12)?,
            err_corr_d: self.a(13)?,
            entry_rate: self.a(14)?,
            thr_lookahead: self.a(15)?,
            thr_min: self.a(17)?,
            thr_boost: self.a(18)?,
            yaw_accel: self.a(19)?,
            lookahead: self.a(20)?,
            path_scale: self.a(21)?,
            ke_rudd: self.a(26)?,
            ke_rudd_lookahead: self.a(27)?,
            alt_abort: self.a(28)?,
            airspeed_min: read(&self.airspeed_min)?,
            scaling_speed: read(&self.scaling_speed)?,
            airspeed_cruise: read(&self.airspeed_cruise)?,
        })
    }
}

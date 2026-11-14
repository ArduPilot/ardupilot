// SPDX-License-Identifier: GPL-3.0-or-later
//! Native counterpart of AP_Scripting/applets/arming-checks.lua.
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Error, Result};

use core::fmt::{self, Write};

ap_rust_sdk::module!(run);

ap_rust_sdk::param_table! {
    pub struct ValuesTable, enum ValuesParam;
    key = 179, prefix = "ARM_V_";
    size = 15;

    // @Param: ARM_V_ALT_LEGAL
    // @DisplayName: Legal max altitude
    // @Description: Legal max altitude for UAV/RPAS/drones in your jurisdiction
    // @Units: m
    // @User: Standard
    AltLegal = 1, "ALT_LEGAL", f32: 120.0;
}

ap_rust_sdk::param_table! {
    pub struct PlaneTable, enum PlaneParam;
    key = 173, prefix = "ARM_P_";
    size = 10;

    // @Param: ARM_P_Q_FS_LAND
    // @DisplayName: Warn if Q failsafe will land
    // @Description: Notify the user that on failsafe a QuadPlan will land. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    QFsLand = 1, "Q_FS_LAND", f32: NOTICE;

    // @Param: ARM_P_Q_FS_RTL
    // @DisplayName: Warn if Q failsafe will QRTL
    // @Description: Notify the user that on failsafe a QuadPlan will QRTL. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    QFsRtl = 2, "Q_FS_RTL", f32: NOTICE;

    // @Param: ARM_P_AIRSPEED
    // @DisplayName: Check AIRSPEED_ parameters
    // @Description: Validate that AIRSPEED_STALL(if set) < MIN < CRUISE < MAX d. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    Airspeed = 3, "AIRSPEED", f32: ERROR;

    // @Param: ARM_P_STALL
    // @DisplayName: AIRSPEED_MIN should be 25% above STALL
    // @Description: Validate that AIRSPEED_MIN is at least 25% above AIRSPEED_STALL(if set). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    Stall = 4, "STALL", f32: INFO;

    // @Param: ARM_P_SCALING
    // @DisplayName: SCALING_SPEED valid
    // @Description: Validate that SCALING_SPEED is within 20% of AIRSPEED_CRUISE. If SCALING_SPEED changes the vehicle may need to be retuned. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    Scaling = 5, "SCALING", f32: ERROR;

    // @Param: ARM_P_RTL_ALT
    // @DisplayName: RTL_ALTITUDE should be a valid value
    // @Description: RTL_ALTITITUDE should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    RtlAlt = 6, "RTL_ALT", f32: ERROR;

    // @Param: ARM_P_QRTL_ALT
    // @DisplayName: Q_RTL_ALT should be a valid value
    // @Description: Q_RTL_ALT should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    QrtlAlt = 7, "QRTL_ALT", f32: ERROR;
}

ap_rust_sdk::param_table! {
    pub struct CopterTable, enum CopterParam;
    key = 172, prefix = "ARM_C_";
    size = 10;

    // @Param: ARM_C_RTL_ALT
    // @DisplayName: RTL_ALT_M should be a valid value
    // @Description: RTL_ALT_M should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    RtlAlt = 1, "RTL_ALT", f32: ERROR;
}

ap_rust_sdk::param_table! {
    pub struct GlobalTable, enum GlobalParam;
    key = 170, prefix = "ARM_";
    size = 20;

    // @Param: ARM_SYSID
    // @DisplayName: MAV_SYSID must be set
    // @Description: Check that MAV_SYSID (or SYDID_THISMAV) has been set. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    Sysid = 1, "SYSID", f32: WARNING;

    // @Param: ARM_FOLL_SYSID
    // @DisplayName: FOLL_SYSID must be set
    // @Description: If FOLL_ENABLE = 1, check that FOLL_SYSID has been set. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    FollSysid = 2, "FOLL_SYSID", f32: ERROR;

    // @Param: ARM_FOLL_SYSID_X
    // @DisplayName: Vehicle should not follow itself
    // @Description: If FOLL_ENABLE = 1, check that FOLL_SYSID is different to MAV_SYSID. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    FollSysidX = 3, "FOLL_SYSID_X", f32: ERROR;

    // @Param: ARM_FOLL_OFS_DEF
    // @DisplayName: Follow Offsets defaulted
    // @Description: Follow offsets should not be left as default (zero) if FOLL_ENABLE = 1. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    FollOfsDef = 4, "FOLL_OFS_DEF", f32: ERROR;

    // @Param: ARM_MNTX_SYSID
    // @DisplayName: Follow and Mount should follow the same vehicle
    // @Description: If FOLL_ENABLE = 1 and MNTx_SYSID_DEFLT is set, check that FOLL_SYSID is equal MNTx. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    MntxSysid = 5, "MNTX_SYSID", f32: WARNING;

    // @Param: ARM_RTL_CLIMB
    // @DisplayName: RTL_CLIMB_MIN should be a valid value
    // @Description: RTL_CLIMB_MIN should be < 120m (400ft). 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    RtlClimb = 6, "RTL_CLIMB", f32: WARNING;

    // @Param: ARM_ESTOP
    // @DisplayName: Motors EStopped
    // @Description: Emergency Stop disables arming. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    Estop = 7, "ESTOP", f32: ERROR;

    // @Param: ARM_FENCE
    // @DisplayName: Fence not enabled
    // @Description: Fences loaded but no fence enabled. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    Fence = 8, "FENCE", f32: WARNING;

    // @Param: ARM_RALLY
    // @DisplayName: Rally too far
    // @Description: Rally Point more than RALLY_LIMIT_KM kilometers away. 3 or less to prevent arming. -1 to disable.
    // @Values: -1:Disabled,0:Emergency(PreArm),1:Alert(PreArm),2:Critical(PreArm),3:Error(PreArm),4:Warning,5:Notice,6:Info,7:Debug
    // @User: Standard
    Rally = 9, "RALLY", f32: WARNING;
}

const SCRIPT_VERSION: &str = "4.7.0-022";
const REFRESH_DELAY_MS: u32 = 500;
const INITIAL_DELAY_MS: u32 = 20_000;
const COPTER: u8 = 2;
const PLANE: u8 = 3;

const ERROR: i32 = 3;
const WARNING: i32 = 4;
const NOTICE: i32 = 5;
const INFO: i32 = 6;

#[derive(Clone, Copy)]
enum CheckKind {
    SysId,
    FollowSysId,
    FollowNotSelf,
    FollowOffsets,
    MountSysId,
    RtlClimb,
    EmergencyStop,
    Fence,
    Rally,
    CopterRtlAltitude,
    QLand,
    QRtl,
    Airspeed,
    StallSpeed,
    ScalingSpeed,
    PlaneRtlAltitude,
    QRtlAltitude,
}

#[derive(Clone, Copy)]
struct Check {
    table: u8,
    prefix: &'static str,
    definition: ap_rust_sdk::ParamDef,
    text: &'static str,
    initial: Option<bool>,
    kind: CheckKind,
}

const GLOBAL_CHECKS: [Check; 9] = [
    check(GlobalTable::KEY, GlobalTable::PREFIX, GlobalTable::definition(GlobalParam::Sysid), "MAV_SYSID not set", None, CheckKind::SysId),
    check(GlobalTable::KEY, GlobalTable::PREFIX, GlobalTable::definition(GlobalParam::FollSysid), "FOLL_SYSID not set", Some(true), CheckKind::FollowSysId),
    check(GlobalTable::KEY, GlobalTable::PREFIX, GlobalTable::definition(GlobalParam::FollSysidX), "FOLL_SYSID == MAV_SYSID", Some(true), CheckKind::FollowNotSelf),
    check(GlobalTable::KEY, GlobalTable::PREFIX, GlobalTable::definition(GlobalParam::FollOfsDef), "FOLL_OFS_[XYZ] = 0", Some(true), CheckKind::FollowOffsets),
    check(GlobalTable::KEY, GlobalTable::PREFIX, GlobalTable::definition(GlobalParam::MntxSysid), "MNTx_SYSID != FOLL", Some(true), CheckKind::MountSysId),
    check(GlobalTable::KEY, GlobalTable::PREFIX, GlobalTable::definition(GlobalParam::RtlClimb), "RTL_CLIMB_MIN too high", Some(true), CheckKind::RtlClimb),
    check(GlobalTable::KEY, GlobalTable::PREFIX, GlobalTable::definition(GlobalParam::Estop), "Motors EStopped", Some(true), CheckKind::EmergencyStop),
    check(GlobalTable::KEY, GlobalTable::PREFIX, GlobalTable::definition(GlobalParam::Fence), "Fence not enabled", Some(true), CheckKind::Fence),
    check(GlobalTable::KEY, GlobalTable::PREFIX, GlobalTable::definition(GlobalParam::Rally), "Rally too far", Some(true), CheckKind::Rally),
];

const PLANE_CHECKS: [Check; 7] = [
    check(PlaneTable::KEY, PlaneTable::PREFIX, PlaneTable::definition(PlaneParam::QFsLand), "Q will land on failsafe", Some(true), CheckKind::QLand),
    check(PlaneTable::KEY, PlaneTable::PREFIX, PlaneTable::definition(PlaneParam::QFsRtl), "Q will RTL on failsafe", Some(true), CheckKind::QRtl),
    check(PlaneTable::KEY, PlaneTable::PREFIX, PlaneTable::definition(PlaneParam::Airspeed), "stall < min < crs < max", Some(true), CheckKind::Airspeed),
    check(PlaneTable::KEY, PlaneTable::PREFIX, PlaneTable::definition(PlaneParam::Stall), "Min Speed not 25% above stall", Some(true), CheckKind::StallSpeed),
    check(PlaneTable::KEY, PlaneTable::PREFIX, PlaneTable::definition(PlaneParam::Scaling), "Scaling spd >< 20% of CRUISE", Some(true), CheckKind::ScalingSpeed),
    check(PlaneTable::KEY, PlaneTable::PREFIX, PlaneTable::definition(PlaneParam::RtlAlt), "RTL_ALTITUDE too high", None, CheckKind::PlaneRtlAltitude),
    check(PlaneTable::KEY, PlaneTable::PREFIX, PlaneTable::definition(PlaneParam::QrtlAlt), "Q_RTL_ALT too high", None, CheckKind::QRtlAltitude),
];

const COPTER_CHECKS: [Check; 1] = [check(CopterTable::KEY, CopterTable::PREFIX, CopterTable::definition(CopterParam::RtlAlt), "RTL_ALT_M too high", None, CheckKind::CopterRtlAltitude)];

const fn check(
    table: u8,
    prefix: &'static str,
    definition: ap_rust_sdk::ParamDef,
    text: &'static str,
    initial: Option<bool>,
    kind: CheckKind,
) -> Check {
    Check { table, prefix, definition, text, initial, kind }
}

struct Text<const N: usize> {
    bytes: [u8; N],
    length: usize,
    truncate: bool,
}

impl<const N: usize> Text<N> {
    fn new(args: fmt::Arguments<'_>) -> Result<Self> {
        let mut text = Self { bytes: [0; N], length: 0, truncate: false };
        text.write_fmt(args).map_err(|_| Error::Invalid)?;
        Ok(text)
    }

    fn truncated(args: fmt::Arguments<'_>) -> Self {
        let mut text = Self { bytes: [0; N], length: 0, truncate: true };
        let _ = text.write_fmt(args);
        text
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.length]).unwrap()
    }
}

impl<const N: usize> Write for Text<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let available = N - self.length;
        if value.len() > available && !self.truncate {
            return Err(fmt::Error);
        }
        let count = value.len().min(available);
        self.bytes[self.length..self.length + count].copy_from_slice(&value.as_bytes()[..count]);
        self.length += count;
        Ok(())
    }
}

fn retry<T>(context: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
    let start = context.time_us();
    loop {
        match call() {
            Err(Error::Busy) if context.time_us().saturating_sub(start) < 100_000 => context.sleep_ms(1)?,
            result => return result,
        }
    }
}

fn message(context: &Context<'_>, severity: i32, args: fmt::Arguments<'_>) -> Result {
    let text = Text::<128>::new(args)?;
    retry(context, || context.gcs().send_text(MavSeverity(severity), text.as_str()))
}

fn parameter(context: &Context<'_>, name: &str) -> Result<f32> {
    let value = retry(context, || context.param().get(name))?.ok_or(Error::Invalid)?;
    if value.is_finite() { Ok(value) } else { Err(Error::Invalid) }
}

fn optional_parameter(context: &Context<'_>, name: &str) -> Result<Option<f32>> {
    let value = retry(context, || context.param().get(name))?;
    if value.is_none_or(f32::is_finite) { Ok(value) } else { Err(Error::Invalid) }
}

fn full_name(check: Check) -> Result<Text<17>> {
    Text::new(format_args!("{}{}", check.prefix, check.definition.name))
}

fn add_checks(context: &Context<'_>, checks: &[Check]) -> Result {
    for item in checks {
        item.definition.register(context, item.table)?;
    }
    Ok(())
}

fn severity_text(severity: i32) -> Result<&'static str> {
    match severity {
        0 => Ok("emrg"),
        1 => Ok("alrt"),
        2 => Ok("crit"),
        3 => Ok("fail"),
        4 => Ok("warn"),
        5 => Ok("note"),
        6 => Ok("info"),
        7 => Ok("debg"),
        _ => Err(Error::Invalid),
    }
}

fn evaluate(context: &Context<'_>, kind: CheckKind, altitude_limit: f32, sysid_name: &str) -> Result<bool> {
    let follow_enabled = || parameter(context, "FOLL_ENABLE").map(|value| value == 1.0);
    match kind {
        CheckKind::SysId => Ok(parameter(context, sysid_name)? > 1.0),
        CheckKind::FollowSysId => Ok(!follow_enabled()? || parameter(context, "FOLL_SYSID")? != 0.0),
        CheckKind::FollowNotSelf => {
            let follow = parameter(context, "FOLL_SYSID")?;
            Ok(!follow_enabled()? || follow <= 0.0 || follow != parameter(context, sysid_name)?)
        }
        CheckKind::FollowOffsets => Ok(!follow_enabled()? || parameter(context, "FOLL_OFS_X")? != 0.0 ||
            parameter(context, "FOLL_OFS_Y")? != 0.0 || parameter(context, "FOLL_OFS_Z")? != 0.0),
        CheckKind::MountSysId => {
            if !follow_enabled()? {
                return Ok(true);
            }
            let follow = parameter(context, "FOLL_SYSID")?;
            for index in 1..=2 {
                let type_name = Text::<17>::new(format_args!("MNT{index}_TYPE"))?;
                if optional_parameter(context, type_name.as_str())?.unwrap_or(0.0) == 0.0 {
                    continue;
                }
                let sysid_name = Text::<17>::new(format_args!("MNT{index}_SYSID_DFLT"))?;
                let sysid = parameter(context, sysid_name.as_str())?;
                if sysid > 0.0 && sysid != follow {
                    return Ok(false);
                }
            }
            Ok(true)
        }
        CheckKind::RtlClimb => {
            let value = match optional_parameter(context, "RTL_CLIMB_MIN")? {
                Some(value) => Some(value),
                None => optional_parameter(context, "RTL_CLIMB_MIN_M")?,
            };
            Ok(value.is_none_or(|value| value <= altitude_limit))
        }
        CheckKind::EmergencyStop => Ok(!retry(context, || context.srv_channels().get_emergency_stop())?),
        CheckKind::Fence => {
            let fence_type = parameter(context, "FENCE_TYPE")? as u32;
            let present = fence_type & (8 | 2 | 1) != 0 ||
                (fence_type & 4 != 0 && parameter(context, "FENCE_TOTAL")? > 0.0);
            Ok(!present || parameter(context, "FENCE_ENABLE")? == 1.0 ||
                parameter(context, "FENCE_AUTOENABLE")? > 0.0)
        }
        CheckKind::Rally => {
            let maximum = parameter(context, "RALLY_LIMIT_KM")? * 1000.0;
            if maximum <= 0.0 {
                return Ok(true);
            }
            let mut home = retry(context, || context.ahrs().get_home())?;
            for index in 0..=u8::MAX {
                let mut rally = match retry(context, || context.rally().get_rally_location(index)) {
                    Ok(Some(location)) => location,
                    Ok(None) | Err(Error::Unavailable) => return Ok(true),
                    Err(error) => return Err(error),
                };
                if retry(context, || home.get_distance(context, &mut rally))? > maximum {
                    return Ok(false);
                }
            }
            Ok(true)
        }
        CheckKind::CopterRtlAltitude => Ok(parameter(context, "RTL_ALT_M")? <= altitude_limit),
        CheckKind::QLand | CheckKind::QRtl => {
            if parameter(context, "Q_ENABLE")? != 1.0 {
                return Ok(true);
            }
            let options = parameter(context, "Q_OPTIONS")? as u32;
            let selected = options & (1 << 5) != 0 || options & (1 << 20) != 0;
            Ok(match kind { CheckKind::QLand => selected, _ => !selected })
        }
        CheckKind::Airspeed => {
            let stall = parameter(context, "AIRSPEED_STALL")?;
            let minimum = parameter(context, "AIRSPEED_MIN")?;
            let cruise = parameter(context, "AIRSPEED_CRUISE")?;
            let maximum = parameter(context, "AIRSPEED_MAX")?;
            Ok((stall <= 0.0 || minimum >= stall) && cruise >= minimum && maximum >= cruise)
        }
        CheckKind::StallSpeed => {
            let stall = parameter(context, "AIRSPEED_STALL")?;
            Ok(stall <= 0.0 || parameter(context, "AIRSPEED_MIN")? <= stall * 1.25)
        }
        CheckKind::ScalingSpeed => {
            let scaling = parameter(context, "SCALING_SPEED")?;
            let cruise = parameter(context, "AIRSPEED_CRUISE")?;
            Ok(scaling > 0.0 && cruise > 0.0 && scaling >= cruise * 0.8 && scaling <= cruise * 1.2)
        }
        CheckKind::PlaneRtlAltitude => Ok(parameter(context, "RTL_ALTITUDE")? <= altitude_limit),
        CheckKind::QRtlAltitude => Ok(parameter(context, "Q_ENABLE")? != 1.0 ||
            parameter(context, "Q_RTL_ALT")? <= altitude_limit),
    }
}

fn run_checks(
    context: &Context<'_>,
    checks: &[Check],
    states: &mut [Option<bool>],
    offset: usize,
    altitude_limit: f32,
    sysid_name: &str,
    auth_id: u8,
    validated: &mut bool,
) -> Result {
    for (index, item) in checks.iter().enumerate() {
        let name = full_name(*item)?;
        let severity = parameter(context, name.as_str())? as i32;
        if severity == -1 {
            continue;
        }
        let passed = evaluate(context, item.kind, altitude_limit, sysid_name)?;
        let changed = states[offset + index] != Some(passed);
        states[offset + index] = Some(passed);
        let mut raise_prearm = !passed;
        if changed {
            if passed {
                message(context, INFO, format_args!("ArmCk: clear: {}", item.text))?;
            } else if severity > ERROR {
                message(context, severity, format_args!("ArmCk: {}: {}", severity_text(severity)?, item.text))?;
                raise_prearm = false;
            }
        } else if !passed && severity > ERROR {
            raise_prearm = false;
        }
        if raise_prearm {
            *validated = false;
            let failure = Text::<41>::truncated(format_args!(
                "ArmCk: {}: {}", severity_text(severity)?, item.text
            ));
            retry(context, || context.arming().set_aux_auth_failed(auth_id, failure.as_str()))?;
        }
    }
    Ok(())
}

fn run(context: &Context<'_>) -> Result {
    GlobalTable::register_table(context)?;
    CopterTable::register_table(context)?;
    PlaneTable::register_table(context)?;
    ValuesTable::register_table(context)?;
    let firmware = retry(context, || context.fw_version().r#type())?;
    add_checks(context, &GLOBAL_CHECKS)?;
    let vehicle_checks: &[Check] = match firmware {
        COPTER => &COPTER_CHECKS,
        PLANE => &PLANE_CHECKS,
        _ => &[],
    };
    add_checks(context, vehicle_checks)?;
    ValuesTable::definition(ValuesParam::AltLegal).register(context, ValuesTable::KEY)?;
    let sysid_name = if optional_parameter(context, "SYSID_THISMAV")?.is_some() {
        "SYSID_THISMAV"
    } else {
        "MAV_SYSID"
    };
    let auth_id = retry(context, || context.arming().get_aux_auth_id())?.ok_or(Error::NoMemory)?;
    let mut states = [None; GLOBAL_CHECKS.len() + PLANE_CHECKS.len()];
    for (index, item) in GLOBAL_CHECKS.iter().chain(vehicle_checks).enumerate() {
        states[index] = item.initial;
    }

    match context.sleep_ms(INITIAL_DELAY_MS) {
        Ok(()) => {}
        Err(Error::Stopped) => return Ok(()),
        Err(_) => {}
    }
    let _ = message(context, INFO, format_args!("Arming Checks {SCRIPT_VERSION} loaded"));
    loop {
        let delay = match (|| -> Result<u32> {
            if retry(context, || context.arming().is_armed())? {
                return Ok(REFRESH_DELAY_MS * 20);
            }
            let altitude_limit = optional_parameter(context, "ARM_V_ALT_LEGAL")?.unwrap_or(120.0);
            let mut validated = true;
            run_checks(context, &GLOBAL_CHECKS, &mut states, 0, altitude_limit,
                       sysid_name, auth_id, &mut validated)?;
            run_checks(context, vehicle_checks, &mut states, GLOBAL_CHECKS.len(), altitude_limit,
                       sysid_name, auth_id, &mut validated)?;
            if validated {
                retry(context, || context.arming().set_aux_auth_passed(auth_id))?;
            }
            Ok(REFRESH_DELAY_MS)
        })() {
            Ok(delay) => delay,
            Err(Error::Stopped) => return Ok(()),
            Err(_) => {
                let _ = retry(context, || {
                    context.arming().set_aux_auth_failed(auth_id, "ArmCk: internal error")
                });
                REFRESH_DELAY_MS
            }
        };
        match context.sleep_ms(delay) {
            Ok(()) => {}
            Err(Error::Stopped) => return Ok(()),
            Err(_) => {}
        }
    }
}

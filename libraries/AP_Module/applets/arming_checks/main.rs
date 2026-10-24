// SPDX-License-Identifier: GPL-3.0-or-later
//! Native counterpart of AP_Scripting/applets/arming-checks.lua.
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Error, Result};
use core::fmt::{self, Write};

ap_rust_sdk::module!(run);

const SCRIPT_VERSION: &str = "4.7.0-022";
const REFRESH_DELAY_MS: u32 = 500;
const INITIAL_DELAY_MS: u32 = 20_000;
const GLOBAL_TABLE: u8 = 170;
const COPTER_TABLE: u8 = 172;
const PLANE_TABLE: u8 = 173;
const VALUES_TABLE: u8 = 179;
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
    index: u8,
    suffix: &'static str,
    text: &'static str,
    severity: i32,
    initial: Option<bool>,
    kind: CheckKind,
}

const GLOBAL_CHECKS: [Check; 9] = [
    check(GLOBAL_TABLE, "ARM_", 1, "SYSID", "MAV_SYSID not set", WARNING, None, CheckKind::SysId),
    check(GLOBAL_TABLE, "ARM_", 2, "FOLL_SYSID", "FOLL_SYSID not set", ERROR, Some(true), CheckKind::FollowSysId),
    check(GLOBAL_TABLE, "ARM_", 3, "FOLL_SYSID_X", "FOLL_SYSID == MAV_SYSID", ERROR, Some(true), CheckKind::FollowNotSelf),
    check(GLOBAL_TABLE, "ARM_", 4, "FOLL_OFS_DEF", "FOLL_OFS_[XYZ] = 0", ERROR, Some(true), CheckKind::FollowOffsets),
    check(GLOBAL_TABLE, "ARM_", 5, "MNTX_SYSID", "MNTx_SYSID != FOLL", WARNING, Some(true), CheckKind::MountSysId),
    check(GLOBAL_TABLE, "ARM_", 6, "RTL_CLIMB", "RTL_CLIMB_MIN too high", WARNING, Some(true), CheckKind::RtlClimb),
    check(GLOBAL_TABLE, "ARM_", 7, "ESTOP", "Motors EStopped", ERROR, Some(true), CheckKind::EmergencyStop),
    check(GLOBAL_TABLE, "ARM_", 8, "FENCE", "Fence not enabled", WARNING, Some(true), CheckKind::Fence),
    check(GLOBAL_TABLE, "ARM_", 9, "RALLY", "Rally too far", WARNING, Some(true), CheckKind::Rally),
];

const PLANE_CHECKS: [Check; 7] = [
    check(PLANE_TABLE, "ARM_P_", 1, "Q_FS_LAND", "Q will land on failsafe", NOTICE, Some(true), CheckKind::QLand),
    check(PLANE_TABLE, "ARM_P_", 2, "Q_FS_RTL", "Q will RTL on failsafe", NOTICE, Some(true), CheckKind::QRtl),
    check(PLANE_TABLE, "ARM_P_", 3, "AIRSPEED", "stall < min < crs < max", ERROR, Some(true), CheckKind::Airspeed),
    check(PLANE_TABLE, "ARM_P_", 4, "STALL", "Min Speed not 25% above stall", INFO, Some(true), CheckKind::StallSpeed),
    check(PLANE_TABLE, "ARM_P_", 5, "SCALING", "Scaling spd >< 20% of CRUISE", ERROR, Some(true), CheckKind::ScalingSpeed),
    check(PLANE_TABLE, "ARM_P_", 6, "RTL_ALT", "RTL_ALTITUDE too high", ERROR, None, CheckKind::PlaneRtlAltitude),
    check(PLANE_TABLE, "ARM_P_", 7, "QRTL_ALT", "Q_RTL_ALT too high", ERROR, None, CheckKind::QRtlAltitude),
];

const COPTER_CHECKS: [Check; 1] = [check(
    COPTER_TABLE,
    "ARM_C_",
    1,
    "RTL_ALT",
    "RTL_ALT_M too high",
    ERROR,
    None,
    CheckKind::CopterRtlAltitude,
)];

const fn check(
    table: u8,
    prefix: &'static str,
    index: u8,
    suffix: &'static str,
    text: &'static str,
    severity: i32,
    initial: Option<bool>,
    kind: CheckKind,
) -> Check {
    Check { table, prefix, index, suffix, text, severity, initial, kind }
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
    Text::new(format_args!("{}{}", check.prefix, check.suffix))
}

fn add_checks(context: &Context<'_>, checks: &[Check]) -> Result {
    for item in checks {
        if !retry(context, || {
            context.param().add_param(item.table, item.index, item.suffix, item.severity as f32)
        })? {
            return Err(Error::Host);
        }
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
    for (key, prefix, count) in [
        (GLOBAL_TABLE, "ARM_", 20),
        (COPTER_TABLE, "ARM_C_", 10),
        (PLANE_TABLE, "ARM_P_", 10),
        (VALUES_TABLE, "ARM_V_", 15),
    ] {
        if !retry(context, || context.param().add_table(key, prefix, count))? {
            return Err(Error::Host);
        }
    }
    let firmware = retry(context, || context.fw_version().r#type())?;
    add_checks(context, &GLOBAL_CHECKS)?;
    let vehicle_checks: &[Check] = match firmware {
        COPTER => &COPTER_CHECKS,
        PLANE => &PLANE_CHECKS,
        _ => &[],
    };
    add_checks(context, vehicle_checks)?;
    if !retry(context, || context.param().add_param(VALUES_TABLE, 1, "ALT_LEGAL", 120.0))? {
        return Err(Error::Host);
    }
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

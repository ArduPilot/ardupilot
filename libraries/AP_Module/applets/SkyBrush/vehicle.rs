// SPDX-License-Identifier: GPL-3.0-or-later
//! Vehicle control bindings used by the show (custom-mode controller requests,
//! forced disarm, fence upload). Isolated here so the stage machine stays binding-agnostic.

use ap_rust_sdk::{
    enums::{ApArmingMethod, ModeReason},
    Context, CustomMode, CustomModeRequest, Error, Location, Result, Vector3f, CUSTOM_MODE_ARMING_GATED,
    CUSTOM_MODE_EKF_FS_DISARM, CUSTOM_MODE_NO_EXTERNAL_GUIDED,
};
use core::num::NonZeroU16;

pub struct Mode {
    handle: CustomMode,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Request {
    Guided,
    Takeoff,
    Land,
    Rtl,
    Loiter,
}

#[derive(Default, Clone, Copy)]
pub struct Status {
    /// the current request generation has been applied by the main thread
    pub applied: bool,
    pub init_ok: bool,
    /// the current request has completed (latched)
    pub complete: bool,
    /// a GCS takeoff command arrived while in the mode
    pub takeoff_event: bool,
    pub owner_lost: bool,
}

pub const DISARM_FENCEBREACH: ApArmingMethod = ApArmingMethod::FENCEBREACH;
pub const DISARM_EKFFAILSAFE: ApArmingMethod = ApArmingMethod::EKFFAILSAFE;
pub const REASON_FENCE_BREACHED: ModeReason = ModeReason::FENCE_BREACHED;
const DISARMDELAY: i32 = 17;
// owner supervision: land if the module stops ticking for this long. A
// stopped module is caught at once; this only catches a hung one, so allow
// for scheduling delays, which SITL speedup magnifies.
const SUPERVISION_MS: u16 = 3000;
const BASE_OPTIONS: u8 = CUSTOM_MODE_NO_EXTERNAL_GUIDED | CUSTOM_MODE_ARMING_GATED;

pub fn register(c: &Context<'_>, number: u8) -> Result<Mode> {
    let handle = c
        .vehicle()
        .register_custom_mode(number, "DRONE_SHOW", "SHOW")?
        .ok_or(Error::Busy)?;
    handle.set_options(c, BASE_OPTIONS)?;
    handle.set_allow_entry(c, true)?;
    Ok(Mode { handle })
}

/// Enable owner supervision once startup work is done.
pub fn start_supervision(c: &Context<'_>, m: &Mode) -> Result {
    m.handle.keepalive(c)?;
    m.handle.set_supervision(c, SUPERVISION_MS)
}

/// Returns the request generation, 0 if the mode is not active.
pub fn request(c: &Context<'_>, m: &Mode, r: Request, alt: f32) -> Result<u16> {
    let req = match r {
        Request::Guided => CustomModeRequest::Guided,
        Request::Takeoff => CustomModeRequest::Takeoff,
        Request::Land => CustomModeRequest::Land,
        Request::Rtl => CustomModeRequest::Rtl,
        Request::Loiter => CustomModeRequest::Loiter,
    };
    Ok(m.handle.set_request(c, req, alt)?.map_or(0, |g| g.get()))
}

pub fn status(c: &Context<'_>, m: &Mode, gen: u16) -> Result<Status> {
    let st = m.handle.status(c)?;
    let (applied, complete) = match NonZeroU16::new(gen) {
        Some(g) => (st.applied(g), st.completed(g)),
        None => (false, false),
    };
    Ok(Status {
        applied,
        init_ok: st.init_ok,
        complete,
        takeoff_event: st.takeoff_event,
        owner_lost: st.owner_lost,
    })
}

pub fn ack_takeoff_event(c: &Context<'_>, m: &Mode) -> Result<bool> {
    m.handle.ack_takeoff_event(c)
}

pub fn set_arming_ready(c: &Context<'_>, m: &Mode, ready: bool) -> Result {
    m.handle.set_arming_ready(c, ready)
}

pub fn set_ekf_disarm(c: &Context<'_>, m: &Mode, enable: bool) -> Result {
    let opts = if enable { BASE_OPTIONS | CUSTOM_MODE_EKF_FS_DISARM } else { BASE_OPTIONS };
    m.handle.set_options(c, opts)
}

pub fn keepalive(c: &Context<'_>, m: &Mode) -> Result {
    m.handle.keepalive(c)
}

/// Guided target, only accepted while our mode is the current mode.
pub fn set_target_pva(
    c: &Context<'_>,
    m: &Mode,
    pos: &Vector3f,
    vel: &Vector3f,
    acc: &Vector3f,
    yaw_deg: f32,
    yaw_rate_dps: f32,
) -> Result<bool> {
    m.handle.set_target_posvelaccel(c, pos, vel, acc, true, yaw_deg, true, yaw_rate_dps, false)
}

/// Ask the main thread to arm, as arming may delay while the log opens;
/// arm false cancels a pending request.
pub fn request_arm(c: &Context<'_>, m: &Mode, arm: bool) -> Result<bool> {
    m.handle.request_arm(c, arm)
}

/// Disarm in flight without arming checks.
pub fn force_disarm(c: &Context<'_>, method: ApArmingMethod) -> Result {
    if c.arming().is_armed()? {
        c.arming().disarm_method(method, false)?;
    }
    Ok(())
}

pub fn set_mode_reason(c: &Context<'_>, mode: u8, reason: ModeReason) -> Result<bool> {
    c.vehicle().set_mode_reason(mode, reason)
}

pub fn last_disarm_was_auto(c: &Context<'_>) -> Result<bool> {
    Ok(c.arming().last_disarm_method()?.0 == DISARMDELAY)
}

pub fn write_inclusion_polygon(c: &Context<'_>, points: &[Location]) -> Result<bool> {
    c.fence().write_inclusion_polygon(points)
}

pub fn fence_breach_count(c: &Context<'_>) -> Result<u16> {
    c.fence().get_breach_count()
}

pub fn calibrate_baro(c: &Context<'_>) -> Result {
    // MAV_CMD_PREFLIGHT_CALIBRATION with param3=1 (baro); refused while armed
    let p = ap_rust_sdk::CommandIntParams { p3: 1.0, ..Default::default() };
    let _ = c.gcs().run_command_int(241, &p);
    Ok(())
}

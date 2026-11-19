// SPDX-License-Identifier: GPL-3.0-or-later
//! Show manager state and the DRONE_SHOW stage machine (ModeDroneShow logic).

use crate::{
    coords::ShowFrame,
    crth,
    fences::{self, BubbleFence, BubbleParams, HardFence, HardFenceParams, HardMsg, RateLimit},
    geofence,
    lights_backends::Led,
    lights_out::{self as lo, LightInputs, Lights},
    loader,
    loader::{LoadConfig, Show},
    logging, mavlink as mav, monitors,
    params::{value, Params, VehicleParams, P},
    show::{controller::secs_to_msec, fmath::hypotf, Vec3},
    telemetry, timing,
    timing::Clock,
    vehicle::{self, Request},
};
use ap_rust_sdk::{enums::MavSeverity, Context, Location, Result, Vector3f};

#[path = "pyro.rs"]
mod pyro;

pub const MODE_NUMBER: u8 = 127;
const MODE_RTL: u8 = 6;
const MODE_LAND: u8 = 9;
const SMALLEST_VALID_AMSL: i32 = -9999999;
const LATEST_TAKEOFF_ATTEMPT_SEC: f32 = 5.0;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Stage {
    Off = 0,
    Init = 1,
    WaitForStartTime = 2,
    Takeoff = 3,
    Performing = 4,
    Rtl = 5,
    Loiter = 6,
    Landing = 7,
    Landed = 8,
    Error = 9,
    TestingLights = 10,
}

const AUTH_REVOKED: u8 = 0;
const AUTH_GRANTED: u8 = 1;
const AUTH_LIGHTS_ONLY: u8 = 3;

const PREFLIGHT_NOT_CONFIGURED: u8 = 1 << 0;
const PREFLIGHT_NOT_AT_TAKEOFF: u8 = 1 << 7;

const OPTION_CORRECT_LANDING: i32 = 1 << 1;

const POST_LOITER: i32 = 0;
const POST_LAND: i32 = 1;
const POST_RTL: i32 = 2;
const POST_RTL_OR_LAND: i32 = 3;

pub struct App<'a, 'h> {
    c: &'a Context<'h>,
    pub params: Params<'a, 'h>,
    vparams: VehicleParams<'a, 'h>,
    mode: vehicle::Mode,
    show: Option<Show>,
    clock: Clock,
    pub stage: Stage,
    stage_started_ms: u32,
    in_mode: bool,
    tentative: Option<ShowFrame>,
    flight: Option<ShowFrame>,
    preflight_failures: u8,
    preflight_checked_ms: u32,
    // a fence write failed after it was acknowledged
    fence_failed: bool,
    motors_started: bool,
    /// we asked the main thread to arm and have not seen it armed yet
    arm_requested: bool,
    prevent_arming_until_ms: u32,
    calibration_done: bool,
    home_set: bool,
    next_after_takeoff: Stage,
    /// heading at arming, as Copter's initial_armed_bearing
    default_yaw_deg: f32,
    was_armed: bool,
    altitude_locked: bool,
    request_gen: u16,
    next_setpoint_ms: u32,
    last_setpoint_ned: Option<Vector3f>,
    last_auth: u8,
    hard: HardFence,
    hard_breach_point: Option<Location>,
    hard_text: RateLimit,
    bubble: BubbleFence,
    bubble_text: RateLimit,
    last_rgb: [u8; 3],
    lights: Lights,
    led: Led,
    tick: u32,
    boot_count: u8,
    ekf_disarm: bool,
    start_time_pending: bool,
    switch_to_show_pending: bool,
    rc_switch_high: bool,
    rc_blocked_until_ms: u32,
    start_from_rc: bool,
    last_status_ms: u32,
    last_fence_log_ms: u32,
    pyro: pyro::Pyro,
    rtcm: [monitors::RtcmRate; 2],
    esc: monitors::EscErrors,
    cal: monitors::CalEvents,
}

fn now_ms(c: &Context<'_>) -> u32 {
    (c.time_us() / 1000) as u32
}

/// GPS instance bindings reject instances before the GPS is detected.
fn gps_value<T: Default>(r: Result<T>) -> Result<T> {
    match r {
        Err(ap_rust_sdk::Error::Invalid) => Ok(T::default()),
        r => r,
    }
}

fn text(c: &Context<'_>, sev: MavSeverity, s: &str) {
    let _ = c.gcs().send_text(sev, s);
}

impl<'a, 'h> App<'a, 'h> {
    pub fn new(c: &'a Context<'h>, mut params: Params<'a, 'h>, mode: vehicle::Mode) -> Result<Self> {
        params.reset_volatile()?;
        let boot_count = c
            .parameter("STAT_BOOTCNT")?
            .and_then(|p| p.get().ok().flatten())
            .unwrap_or(0.0) as u8;
        let mut app = Self {
            c,
            params,
            vparams: VehicleParams::bind(c)?,
            mode,
            show: None,
            clock: Clock::default(),
            stage: Stage::Off,
            stage_started_ms: 0,
            in_mode: false,
            tentative: None,
            flight: None,
            preflight_failures: 0,
            preflight_checked_ms: 0,
            fence_failed: false,
            motors_started: false,
            arm_requested: false,
            prevent_arming_until_ms: 0,
            calibration_done: false,
            home_set: false,
            next_after_takeoff: Stage::Performing,
            default_yaw_deg: 0.0,
            was_armed: false,
            altitude_locked: false,
            request_gen: 0,
            next_setpoint_ms: 0,
            last_setpoint_ned: None,
            last_auth: 0,
            hard: HardFence::default(),
            hard_breach_point: None,
            hard_text: RateLimit::default(),
            bubble: BubbleFence::default(),
            bubble_text: RateLimit::default(),
            last_rgb: [0; 3],
            lights: Lights::default(),
            led: Led::default(),
            tick: 0,
            boot_count,
            ekf_disarm: false,
            start_time_pending: false,
            switch_to_show_pending: false,
            rc_switch_high: false,
            rc_blocked_until_ms: 0,
            start_from_rc: false,
            last_status_ms: 0,
            last_fence_log_ms: 0,
            pyro: pyro::Pyro::default(),
            rtcm: Default::default(),
            esc: Default::default(),
            cal: Default::default(),
        };
        app.clock.use_gps = app.params.get_i32(P::SyncMode) != 0;
        app.update_frame();
        app.reload_show()?;
        crate::report(c, "led", app.led.configure(c, &app.params))?;
        crate::report(c, "pyro", app.update_pyro_instance())?;
        Ok(app)
    }

    pub fn mode(&self) -> &vehicle::Mode {
        &self.mode
    }

    fn load_config(&self) -> Result<LoadConfig> {
        let get = |p, default: f32| value(p).filter(|v| v.is_finite() && *v > 0.0).unwrap_or(default);
        let v = &self.vparams;
        Ok(LoadConfig {
            takeoff_alt_m: self.params.get(P::TakeoffAlt),
            takeoff_speed_ms: get(&v.wp_speed_up, 1.0),
            takeoff_accel_mss: get(&v.wp_accel_z, 1.0),
            spool_time_sec: get(&v.spool_time, 0.5),
            pyro_tcomp_msec: self.params.get_i32(P::PyroTcomp),
        })
    }

    pub fn reload_show(&mut self) -> Result<bool> {
        let cfg = self.load_config()?;
        self.show = loader::load(self.c, &cfg)?;
        let block = self.show.as_ref().and_then(|s| s.light_control.as_deref());
        self.lights.pixel.load_block(block);
        self.publish_arming_ready()?;
        Ok(self.show.is_some())
    }

    fn show_ok(&self) -> bool {
        self.show.as_ref().is_some_and(|s| s.plausible())
    }

    fn origin_set(&self) -> bool {
        self.params.get_i32(P::OriginLat) != 0 || self.params.get_i32(P::OriginLng) != 0
    }

    fn orientation_set(&self) -> bool {
        self.params.get(P::Orientation) >= 0.0
    }

    fn update_frame(&mut self) {
        self.tentative = (self.origin_set() && self.orientation_set()).then(|| {
            let amsl = self.params.get_i32(P::OriginAmsl);
            ShowFrame::new(
                self.params.get_i32(P::OriginLat),
                self.params.get_i32(P::OriginLng),
                amsl,
                amsl >= SMALLEST_VALID_AMSL,
                self.params.get(P::Orientation),
            )
        });
    }

    fn publish_arming_ready(&self) -> Result {
        let ready = self.show_ok() && self.origin_set() && self.orientation_set();
        vehicle::set_arming_ready(self.c, &self.mode, ready)
    }

    fn auth(&self) -> u8 {
        self.params.get_i32(P::StartAuth).clamp(0, 3) as u8
    }

    fn set_stage(&mut self, s: Stage) {
        if s == Stage::Takeoff && self.stage != Stage::Takeoff {
            // the first time axis config usually arrives before arming; log it here
            self.log_screenplay();
        }
        self.stage = s;
        self.stage_started_ms = now_ms(self.c);
        self.altitude_locked = s == Stage::Performing;
        self.preflight_checked_ms = 0;
    }

    fn time_in_stage_ms(&self) -> u32 {
        now_ms(self.c).wrapping_sub(self.stage_started_ms)
    }

    /// GPS time as UNIX epoch usec, 0 if unknown. Week and time of week are
    /// derived from this one host call so they are consistent.
    fn gps_epoch_usec(&self) -> Result<u64> {
        let gps = self.c.gps();
        gps_value(gps.time_epoch_usec(gps.primary_sensor()?))
    }

    /// (GPS week, time of week ms) now
    fn gps_week_tow_ms(&self) -> Result<Option<(u16, u32)>> {
        let usec = self.gps_epoch_usec()?;
        Ok((usec != 0).then(|| timing::unix_ms_to_week_tow_ms(usec / 1000)))
    }

    fn elapsed_usec(&mut self) -> Result<Option<i64>> {
        let raw = self.gps_epoch_usec()?;
        Ok(self.clock.elapsed_usec(raw, self.c.time_us()))
    }

    fn gps_time_ok(&self) -> Result<bool> {
        // time_epoch_usec() is 0 until the GPS week is known
        Ok(self.gps_epoch_usec()? != 0)
    }

    /// Recompute the start time from SHOW_START_TIME/MSEC (GPS mode); it stays
    /// pending until GPS time is valid, as in the fork.
    fn update_start_time(&mut self) -> Result {
        self.start_time_pending = true;
        self.apply_pending_start_time()
    }

    fn apply_pending_start_time(&mut self) -> Result {
        if !self.start_time_pending {
            return Ok(());
        }
        if !self.clock.use_gps || !self.start_time_change_allowed() {
            self.start_time_pending = false;
            return Ok(());
        }
        let tow = self.params.get_i32(P::StartTime);
        if tow < 0 {
            self.clock.start_unix_usec = 0;
            self.start_time_pending = false;
            return Ok(());
        }
        let Some((week, tow_ms)) = self.gps_week_tow_ms()? else {
            return Ok(());
        };
        self.start_time_pending = false;
        let start_ms = tow as i64 * 1000 + self.params.get_i32(P::StartMsec) as i64;
        self.clock.start_unix_usec = timing::resolve_start_unix_usec(start_ms, week, tow_ms);
        Ok(())
    }

    fn start_time_change_allowed(&self) -> bool {
        matches!(self.stage, Stage::Off | Stage::Init | Stage::WaitForStartTime | Stage::Landed)
    }

    fn has_start_time(&self) -> bool {
        if self.clock.use_gps {
            self.clock.start_unix_usec != 0
        } else {
            self.clock.start_internal_usec != 0
        }
    }

    fn param_changes(&mut self) -> Result {
        let ch = self.params.refresh()?;
        if P::OriginLat.changed(ch)
            || P::OriginLng.changed(ch)
            || P::OriginAmsl.changed(ch)
            || P::Orientation.changed(ch)
        {
            self.update_frame();
            self.publish_arming_ready()?;
        }
        if P::SyncMode.changed(ch) {
            self.clock.use_gps = self.params.get_i32(P::SyncMode) != 0;
        }
        if P::StartTime.changed(ch) || P::StartMsec.changed(ch) || P::SyncMode.changed(ch) {
            self.update_start_time()?;
            self.calibration_done = false;
            self.home_set = false;
        }
        let auth = self.auth();
        if auth != self.last_auth {
            self.last_auth = auth;
            self.switch_to_show_pending = auth != AUTH_REVOKED && self.params.get_i32(P::ModeBoot) & 2 != 0;
            if auth != AUTH_REVOKED {
                if self.stage == Stage::WaitForStartTime {
                    self.try_update_home()?;
                }
            }
        }
        Ok(())
    }

    /// SHOW_MODE_BOOT bit1: enter show mode once authorized; retried until the
    /// vehicle accepts it (e.g. once it has a position estimate)
    fn switch_to_show_mode(&mut self) -> Result {
        if !self.switch_to_show_pending {
            return Ok(());
        }
        if self.in_mode || self.c.arming().is_armed()? {
            self.switch_to_show_pending = false;
            return Ok(());
        }
        if self.c.vehicle().set_mode(MODE_NUMBER)? {
            self.switch_to_show_pending = false;
        }
        Ok(())
    }

    pub fn handle_start_config(&mut self, sc: &mav::StartConfig) -> Result {
        if sc.start_time < 0 {
            self.params.set_i32(P::StartTime, -1)?;
            self.params.set(P::StartMsec, 0.0)?;
        } else if (sc.start_time as i64) < timing::GPS_WEEK_SEC {
            self.params.set_i32(P::StartTime, sc.start_time)?;
            self.params.set(P::StartMsec, (sc.msec_offset.unwrap_or(0) & 0x3FF) as f32)?;
        }
        match sc.authorization {
            255 => {}
            a if a <= 3 => self.params.set(P::StartAuth, a as f32)?,
            _ => self.params.set(P::StartAuth, 0.0)?,
        }
        if let (false, Some(ms)) = (self.clock.use_gps, sc.countdown_msec) {
            const WEEK_MS: i32 = 604800000;
            if ms == 0x7FFFFFFF {
            } else if ms < -WEEK_MS {
                self.clock.start_internal_usec = 0;
            } else if (0..WEEK_MS).contains(&ms) && self.start_time_change_allowed() {
                self.clock.start_internal_usec = self.c.time_us() + ms as u64 * 1000;
            }
        }
        self.update_start_time()?;
        self.calibration_done = false;
        self.home_set = false;
        self.param_changes()
    }

    /// MAV_CMD_USER_2 param1=0: set origin, orientation and AMSL at full precision.
    pub fn configure_coordinate_system(&mut self, lat: i32, lng: i32, amsl_mm: i32, orientation: f32) -> Result<bool> {
        const LARGEST_VALID_AMSL: i32 = 10000000;
        if !(-900000000..=900000000).contains(&lat) || !(-1800000000..=1800000000).contains(&lng) {
            return Ok(false);
        }
        if amsl_mm >= LARGEST_VALID_AMSL {
            return Ok(false);
        }
        self.params.set_and_save_i32(P::OriginLat, lat)?;
        self.params.set_and_save_i32(P::OriginLng, lng)?;
        self.params.set_and_save_i32(P::OriginAmsl, amsl_mm)?;
        self.params.set_and_save(P::Orientation, orientation)?;
        self.update_frame();
        self.publish_arming_ready()?;
        Ok(true)
    }

    /// MAV_CMD_USER_1 param1=3: run a command if it is addressed to our group.
    /// y = (group + 1) << 16 | command, z = its param1.
    pub fn group_command(&mut self, cmd: &mav::CommandInt) -> Result<u8> {
        if cmd.y < 0 {
            return Ok(mav::MAV_RESULT_UNSUPPORTED);
        }
        let group_plus_one = cmd.y >> 16;
        if group_plus_one > 0 && group_plus_one - 1 != self.params.get_i32(P::Group) {
            return Ok(mav::MAV_RESULT_ACCEPTED);
        }
        let params = ap_rust_sdk::CommandIntParams {
            p1: cmd.z,
            p2: cmd.param2,
            p3: cmd.param3,
            p4: cmd.param4,
            x: cmd.x,
            y: 0,
            z: 0.0,
            frame: cmd.frame,
        };
        let r = self.c.gcs().run_command_int((cmd.y & 0xFFFF) as u16, &params)?;
        Ok(r.0.clamp(0, 255) as u8)
    }

    /// schedule_delayed_start_after(): only while waiting for the start time.
    fn schedule_delayed_start(&mut self, delay_ms: u32) -> Result<bool> {
        if self.stage != Stage::WaitForStartTime {
            return Ok(false);
        }
        if self.clock.use_gps {
            let Some((_, tow_ms)) = self.gps_week_tow_ms()? else {
                return Ok(false);
            };
            let start = tow_ms.wrapping_add(delay_ms);
            self.params.set_i32(P::StartTime, ((start / 1000) % timing::GPS_WEEK_SEC as u32) as i32)?;
            self.params.set(P::StartMsec, (start % 1000) as f32)?;
            self.update_start_time()?;
            self.calibration_done = false;
            self.home_set = false;
        } else {
            self.clock.start_internal_usec = self.c.time_us() + delay_ms as u64 * 1000;
        }
        Ok(true)
    }

    /// RC aux 248: start the show 10 s after the switch goes high (show mode only).
    fn check_rc_start_switch(&mut self) -> Result {
        const AUX_DRONE_SHOW_START: i32 = 248;
        let now = now_ms(self.c);
        if self.c.notify().get_flags()? & ap_rust_sdk::NotifyFlags::FAILSAFE_RADIO != 0 {
            self.rc_blocked_until_ms = now.wrapping_add(1000);
        }
        let Some(ch) = self
            .c
            .rc()
            .find_channel_for_option(ap_rust_sdk::enums::RcChannelAuxFunc(AUX_DRONE_SHOW_START))?
        else {
            return Ok(());
        };
        let high = ch.get_aux_switch_pos(self.c)? == 2;
        let rising = high && !self.rc_switch_high;
        self.rc_switch_high = high;
        let blocked = (self.rc_blocked_until_ms.wrapping_sub(now) as i32) > 0;
        if rising && self.in_mode && now > 5000 && !blocked && self.schedule_delayed_start(10000)? {
            self.start_from_rc = true;
        }
        Ok(())
    }

    /// A start time set by the RC switch does not survive leaving or re-entering show mode.
    fn clear_rc_start(&mut self) -> Result {
        if self.start_from_rc {
            self.start_from_rc = false;
            self.params.set_i32(P::StartTime, -1)?;
            self.params.set(P::StartMsec, 0.0)?;
            self.clock.start_unix_usec = 0;
            self.clock.start_internal_usec = 0;
        }
        Ok(())
    }

    /// Default stream rates of the selected telemetry profile, as the fork sets at boot.
    pub fn apply_telemetry_profile(&self) -> Result {
        let streams: &[(u32, i32)] = if self.params.get_i32(P::TelProfile) == 1 {
            &[(telemetry::MSG_SYS_STATUS, 1000000)]
        } else {
            &[
                (telemetry::MSG_GLOBAL_POSITION_INT, 500000),
                (telemetry::MSG_SYS_STATUS, 1000000),
                (telemetry::MSG_GPS_RAW_INT, 1000000),
            ]
        };
        for chan in 0..8u8 {
            for (msgid, interval) in streams {
                let _ = self.c.gcs().set_message_interval(chan, *msgid, *interval);
            }
        }
        Ok(())
    }

    pub fn prepare_geofence(&mut self, g: &mav::GeofenceSetup<'_>) -> Result<Option<geofence::Pending>> {
        geofence::prepare(self.c, self.tentative.as_ref(), g)
    }

    pub fn apply_geofence(&mut self, p: &geofence::Pending) -> Result {
        self.fence_failed = !geofence::apply(self.c, p)?;
        Ok(())
    }

    /// TIME_AXIS_CONFIG: new screenplay (time warping, collective RTH scenes).
    pub fn handle_time_axis_config(&mut self, d: &[u8]) -> Result<bool> {
        let Some(h) = crth::header(d) else {
            return Ok(false);
        };
        let allowed = self.start_time_change_allowed();
        let wait_or_takeoff = matches!(self.stage, Stage::WaitForStartTime | Stage::Takeoff);
        let (use_gps, start_usec) = (self.clock.use_gps, self.clock.start_unix_usec);
        let Some(show) = self.show.as_mut() else {
            return Ok(false);
        };
        if !crth::accept_seq(&mut show.last_seq, h.seq) {
            return Ok(true);
        }
        if use_gps && !crth::start_time_move_allowed(&h, start_usec, allowed) {
            return Ok(false);
        }
        if use_gps {
            let (tow, msec) = timing::unix_ms_to_tow(h.start_time_msec);
            self.params.set_i32(P::StartTime, tow)?;
            self.params.set(P::StartMsec, msec as f32)?;
            // the writes update the param cache, so param_changes() would not see them
            self.update_start_time()?;
            self.calibration_done = false;
            self.home_set = false;
        }
        let Some(show) = self.show.as_mut() else {
            return Ok(false);
        };
        let Some((sp, triggers)) = crth::build(show, d, &h, wait_or_takeoff) else {
            return Ok(false);
        };
        show.controller.replace_screenplay(sp);
        let clock_ms = timing::elapsed_msec(self.elapsed_usec()?);
        for t in &triggers {
            let p = t.start;
            logging::crth(self.c, clock_ms, t.rth_start_sec, [p.x / 1000.0, p.y / 1000.0, p.z / 1000.0])?;
        }
        if !triggers.is_empty() {
            text(self.c, MavSeverity::INFO, "Collective RTH triggered");
        }
        self.log_screenplay();
        Ok(true)
    }

    fn log_screenplay(&self) {
        if let Some(show) = self.show.as_ref() {
            let _ = logging::screenplay(self.c, crth::log_seq(show), show.controller.screenplay());
        }
    }

    /// is_collective_rth_triggered(): the GCS added a scene after the main show.
    fn crth_triggered(&self) -> bool {
        self.show.as_ref().is_some_and(|s| s.controller.screenplay().scenes.len() > 1)
    }

    fn is_disarmed_or_landed(&self) -> Result<bool> {
        Ok(!self.c.arming().is_armed()? || !self.c.vehicle().get_likely_flying()?)
    }

    fn try_update_home(&mut self) -> Result<bool> {
        if !self.is_disarmed_or_landed()? {
            return Ok(false);
        }
        let Some(mut loc) = self.c.ahrs().get_location()? else {
            return Ok(false);
        };
        self.c.ahrs().set_home(&mut loc)
    }

    fn time_until_takeoff_sec(&mut self) -> Result<f32> {
        let t = match &self.show {
            Some(s) if s.plausible() => {
                s.controller.screenplay().time_sec_for_tag_and_warped_time(loader::SCENE_TAG_MAIN_SHOW, s.takeoff_time_sec)
            }
            _ => f32::INFINITY,
        };
        let elapsed = self.elapsed_usec()?;
        Ok(t - timing::elapsed_sec(elapsed))
    }

    fn update_preflight(&mut self) -> Result {
        let now = now_ms(self.c);
        if self.preflight_checked_ms != 0 && now.wrapping_sub(self.preflight_checked_ms) < 1000 {
            return Ok(());
        }
        self.preflight_checked_ms = now.max(1);
        // reported only while waiting for the start time, as the fork
        self.preflight_failures = if self.stage == Stage::WaitForStartTime { self.preflight_check()? } else { 0 };
        Ok(())
    }

    /// Show configuration and takeoff position checks, PREFLIGHT_* bits.
    fn preflight_check(&self) -> Result<u8> {
        let mut failures = 0;
        if !self.show_ok() || !self.origin_set() || !self.orientation_set() || self.fence_failed {
            failures |= PREFLIGHT_NOT_CONFIGURED;
        }
        if let (Some(frame), Some(show)) = (self.tentative, self.show.as_ref()) {
            let max = self.params.get(P::TakeoffErr);
            if max > 0.0 {
                let p = show.takeoff_position_mm;
                let mut takeoff = frame.to_global(self.c, [p.x, p.y, p.z])?;
                if let Some(mut here) = self.c.ahrs().get_location()? {
                    let d = here.get_distance(self.c, &mut takeoff)?;
                    if d > max {
                        failures |= PREFLIGHT_NOT_AT_TAKEOFF;
                    }
                }
            }
        }
        Ok(failures)
    }

    /// Checked afresh for every takeoff attempt, whatever the stage.
    fn prepared_to_take_off(&self) -> Result<bool> {
        Ok(self.preflight_check()? == 0 && self.gps_time_ok()?)
    }

    fn request(&mut self, r: Request, alt: f32) -> Result {
        self.request_gen = vehicle::request(self.c, &self.mode, r, alt)?;
        Ok(())
    }

    fn status(&self) -> Result<vehicle::Status> {
        vehicle::status(self.c, &self.mode, self.request_gen)
    }

    // ---- stage transitions ----

    fn initialization_start(&mut self) -> Result {
        self.set_stage(Stage::Init);
        self.request(Request::Guided, 0.0)?;
        if self.is_disarmed_or_landed()? {
            self.wait_start()
        } else {
            self.loiter_start()
        }
    }

    fn wait_start(&mut self) -> Result {
        self.set_stage(Stage::WaitForStartTime);
        self.motors_started = false;
        self.try_update_home()?;
        Ok(())
    }

    fn start_motors(&mut self) -> Result<bool> {
        if self.c.arming().is_armed()? {
            self.arm_requested = false;
            return Ok(true);
        }
        let now = now_ms(self.c);
        if (self.prevent_arming_until_ms.wrapping_sub(now) as i32) > 0 {
            return Ok(false);
        }
        // armed on the main thread; a failed attempt is retried a second later
        self.arm_requested = vehicle::request_arm(self.c, &self.mode, true)?;
        self.prevent_arming_until_ms = now.wrapping_add(1000);
        Ok(false)
    }

    /// Capture the heading on any disarmed to armed transition, whoever armed.
    fn note_armed(&mut self, armed: bool) -> Result {
        if armed && !self.was_armed {
            self.default_yaw_deg = self.c.ahrs().get_yaw_rad()?.to_degrees();
        }
        self.was_armed = armed;
        Ok(())
    }

    fn stop_motors_if_started(&mut self) -> Result {
        if self.arm_requested {
            // cancel a pending request; one already applied counts as started
            vehicle::request_arm(self.c, &self.mode, false)?;
            self.arm_requested = false;
            self.motors_started = true;
        }
        if self.motors_started {
            if self.c.arming().is_armed()? {
                self.c.arming().disarm()?;
            }
            self.motors_started = false;
        }
        Ok(())
    }

    fn disarm_delay_sec(&self) -> Result<f32> {
        let d = value(&self.vparams.disarm_delay).unwrap_or(10.0).clamp(0.0, 127.0);
        Ok(if d > 0.0 { d } else { f32::INFINITY })
    }

    fn wait_run(&mut self) -> Result {
        if self.crth_triggered() {
            return self.landing_start();
        }
        let until = self.time_until_takeoff_sec()?;
        let since = -until;
        if since > LATEST_TAKEOFF_ATTEMPT_SEC + 1.0 {
            return if self.is_disarmed_or_landed()? {
                self.landed_start()
            } else {
                self.loiter_start()
            };
        }
        if until <= 10.0 {
            if !self.calibration_done {
                vehicle::calibrate_baro(self.c)?;
                self.calibration_done = true;
            }
            if !self.home_set {
                if !self.try_update_home()? {
                    text(self.c, MavSeverity::CRITICAL, "Could not set home position, giving up");
                    return self.error_start();
                }
                self.home_set = true;
            }
        } else {
            self.calibration_done = false;
            self.home_set = false;
        }
        if self.auth() == AUTH_GRANTED {
            let armed = self.c.arming().is_armed()?;
            if self.motors_started && !armed && vehicle::last_disarm_was_auto(self.c)? {
                self.motors_started = false;
            }
            let disarm_delay = self.disarm_delay_sec()?;
            if self.motors_started && until > disarm_delay {
                self.stop_motors_if_started()?;
            }
            let lead = if disarm_delay > 0.0 && disarm_delay < 10.0 { 0.8 * disarm_delay } else { 8.0 };
            if until <= lead && !self.motors_started {
                if since < LATEST_TAKEOFF_ATTEMPT_SEC {
                    if self.prepared_to_take_off()? && self.start_motors()? {
                        self.motors_started = true;
                    }
                } else {
                    text(self.c, MavSeverity::CRITICAL, "Failed to start motors, giving up");
                    return self.error_start();
                }
            }
            if until <= 0.0 && self.motors_started && self.home_set {
                self.takeoff_start()?;
            }
        } else {
            self.stop_motors_if_started()?;
            if self.auth() == AUTH_LIGHTS_ONLY {
                let elapsed = self.elapsed_usec()?;
                if timing::elapsed_sec(elapsed) >= 0.0 && elapsed.is_some() {
                    self.set_stage(Stage::TestingLights);
                }
            }
        }
        Ok(())
    }

    fn takeoff_start(&mut self) -> Result {
        if self.c.ahrs().get_location()?.is_none() || !self.prepared_to_take_off()? || self.tentative.is_none() {
            return self.error_start();
        }
        self.flight = self.tentative;
        self.correct_landing()?;
        self.set_stage(Stage::Takeoff);
        self.next_after_takeoff = Stage::Performing;
        let alt = self.params.get(P::TakeoffAlt);
        self.request(Request::Takeoff, alt)
    }

    /// notify_takeoff_started(): land at the actual takeoff point on circular shows.
    fn correct_landing(&mut self) -> Result {
        if self.params.get_i32(P::Options) & OPTION_CORRECT_LANDING == 0 {
            return Ok(());
        }
        let (Some(frame), Some(loc)) = (self.flight, self.c.ahrs().get_location()?) else {
            return Ok(());
        };
        let mut end = frame.from_global(self.c, &loc)?;
        end[2] += self.params.get(P::TakeoffAlt) * 500.0;
        let land_speed = value(&self.vparams.land_speed)
            .filter(|v| *v >= 0.0 && v.is_finite())
            .unwrap_or(1.0);
        if let Some(show) = self.show.as_mut() {
            loader::correct_landing(show, Vec3::new(end[0], end[1], end[2]), land_speed * 1000.0);
        }
        Ok(())
    }

    fn takeoff_run(&mut self) -> Result {
        if self.crth_triggered() {
            return self.landing_start();
        }
        if !self.c.arming().is_armed()? {
            text(self.c, MavSeverity::CRITICAL, "Motors disarmed during takeoff");
            return self.error_start();
        }
        let st = self.status()?;
        if st.applied && !st.init_ok {
            return self.error_start();
        }
        if self.takeoff_completed(&st)? {
            match self.next_after_takeoff {
                Stage::Performing => self.performing_start()?,
                _ => self.loiter_start()?,
            }
        }
        Ok(())
    }

    fn takeoff_completed(&mut self, st: &vehicle::Status) -> Result<bool> {
        if self.next_after_takeoff == Stage::Performing {
            let takeoff_cm = self.params.get(P::TakeoffAlt) * 100.0;
            if let Some(mut loc) = self.c.ahrs().get_location()? {
                if let Some(alt) = loc.get_alt_m(self.c, crate::coords::ALT_ABOVE_HOME)? {
                    if alt * 100.0 >= 0.7 * takeoff_cm {
                        let Some(desired) = self.desired_location()? else {
                            return Ok(false);
                        };
                        let mut desired = desired;
                        if let Some(dalt) = desired.get_alt_m(self.c, crate::coords::ALT_ABOVE_HOME)? {
                            return Ok(dalt >= alt);
                        }
                    }
                }
            }
        }
        Ok(st.complete && self.time_in_stage_ms() > 10000)
    }

    /// Desired global position at the current show time.
    fn desired_location(&mut self) -> Result<Option<Location>> {
        let elapsed = self.elapsed_usec()?;
        let (Some(frame), Some(show)) = (self.flight.or(self.tentative), self.show.as_mut()) else {
            return Ok(None);
        };
        if show.controller.update_time_msec(secs_to_msec(timing::elapsed_sec(elapsed))).is_err() {
            return Ok(None);
        }
        let Some(p) = show.controller.output().position else {
            return Ok(None);
        };
        Ok(Some(frame.to_global(self.c, [p.x, p.y, p.z])?))
    }

    fn performing_start(&mut self) -> Result {
        self.set_stage(Stage::Performing);
        self.next_setpoint_ms = 0;
        self.request(Request::Guided, 0.0)
    }

    fn performance_completed(&self) -> bool {
        match self.stage {
            Stage::Performing => self.show.as_ref().is_some_and(|s| s.controller.has_reached_end()),
            Stage::Landing | Stage::Landed | Stage::Loiter | Stage::Rtl | Stage::Error => true,
            _ => false,
        }
    }

    /// Returns false if the setpoint could not be computed.
    fn send_setpoint(&mut self) -> Result<bool> {
        let elapsed = self.elapsed_usec()?;
        let (Some(frame), Some(show)) = (self.flight, self.show.as_mut()) else {
            return Ok(false);
        };
        if show.controller.update_time_msec(secs_to_msec(timing::elapsed_sec(elapsed))).is_err() {
            return Ok(false);
        }
        let out = show.controller.output();
        let Some(p) = out.position else {
            return Ok(show.controller.has_reached_end());
        };
        let mut yaw = self.default_yaw_deg;
        let mut yaw_rate = 0.0;
        if let (Some(y), Some(r)) = (out.yaw, out.yaw_rate) {
            if y.is_finite() && r.is_finite() {
                yaw = frame.yaw_to_global_deg(y);
                yaw_rate = r;
            }
        }
        let mut vel = Vector3f::default();
        if self.params.get_i32(P::CtrlMode) & 1 != 0 {
            let gain = self.params.get(P::VelFfGain);
            if gain > 0.0 {
                if let Some(v) = out.velocity {
                    let v = frame.vec_to_ned([v.x * gain, v.y * gain, v.z * gain]);
                    if v.x.is_finite() && v.y.is_finite() && v.z.is_finite() {
                        vel = v;
                    }
                }
            }
        }
        let mut acc = Vector3f::default();
        let mut loc = frame.to_global(self.c, [p.x, p.y, p.z])?;
        if self.altitude_locked {
            let takeoff_m = self.params.get(P::TakeoffAlt);
            match loc.get_alt_m(self.c, crate::coords::ALT_ABOVE_HOME)? {
                Some(a) if a < takeoff_m => {
                    loc.set_alt_m(self.c, takeoff_m, crate::coords::ALT_ABOVE_HOME)?;
                    vel.z = 0.0;
                    acc.z = 0.0;
                }
                _ => self.altitude_locked = false,
            }
        }
        let Some(neu) = loc.get_vector_from_origin_neu_m(self.c)? else {
            return Ok(false);
        };
        let pos = Vector3f { x: neu.x, y: neu.y, z: -neu.z };
        if !(pos.x.is_finite() && pos.y.is_finite() && pos.z.is_finite()) {
            text(self.c, MavSeverity::WARNING, "Invalid position command");
            return Ok(false);
        }
        if !vehicle::set_target_pva(self.c, &self.mode, &pos, &vel, &acc, yaw, yaw_rate)? {
            return Ok(false);
        }
        self.last_setpoint_ned = Some(pos);
        Ok(true)
    }

    fn performing_run(&mut self) -> Result {
        if !self.c.arming().is_armed()? {
            text(self.c, MavSeverity::CRITICAL, "Motors disarmed during show");
            return self.error_start();
        }
        let st = self.status()?;
        if !st.applied {
            return Ok(());
        }
        let now = now_ms(self.c);
        if self.next_setpoint_ms == 0 || (now.wrapping_sub(self.next_setpoint_ms) as i32) >= 0 {
            let rate = self.params.get(P::CtrlRate).clamp(1.0, 50.0);
            let dt = ((1000.0 / rate) as u32).clamp(20, 1000);
            self.next_setpoint_ms = if self.next_setpoint_ms == 0 {
                now.wrapping_add(dt)
            } else {
                self.next_setpoint_ms.wrapping_add(dt)
            };
            if !self.send_setpoint()? {
                text(self.c, MavSeverity::ERROR, "Failed to send guided mode command");
                return self.loiter_start();
            }
        }
        if self.performance_completed() {
            match self.end_of_show_action()? {
                POST_RTL => self.rtl_start()?,
                POST_LOITER => self.loiter_start()?,
                _ => self.landing_start()?,
            }
        }
        Ok(())
    }

    fn altitude_above_home_m(&self) -> Result<Option<f32>> {
        match self.c.ahrs().get_location()? {
            Some(mut l) => l.get_alt_m(self.c, crate::coords::ALT_ABOVE_HOME),
            None => Ok(None),
        }
    }

    fn end_of_show_action(&mut self) -> Result<i32> {
        if self.crth_triggered() {
            return Ok(POST_LAND);
        }
        let mut threshold = 1.0;
        let takeoff = self.params.get(P::TakeoffAlt);
        if takeoff > 0.0 {
            threshold = f32::min(threshold, takeoff * 0.5);
        }
        if let Some(a) = self.altitude_above_home_m()? {
            if a < threshold {
                return Ok(POST_LAND);
            }
        }
        Ok(match self.params.get_i32(P::PostAction) {
            POST_LOITER => POST_LOITER,
            POST_RTL => POST_RTL,
            POST_RTL_OR_LAND => {
                let circular = self.show.as_ref().is_some_and(|s| s.circular);
                if circular && self.at_takeoff_position_xy(1.0)? {
                    POST_RTL
                } else {
                    POST_LAND
                }
            }
            _ => POST_LAND,
        })
    }

    fn at_takeoff_position_xy(&self, threshold_m: f32) -> Result<bool> {
        let (Some(frame), Some(show)) = (self.flight.or(self.tentative), self.show.as_ref()) else {
            return Ok(false);
        };
        let p = show.takeoff_position_mm;
        let mut takeoff = frame.to_global(self.c, [p.x, p.y, p.z])?;
        let Some(mut here) = self.c.ahrs().get_location()? else {
            return Ok(false);
        };
        Ok(here.get_distance(self.c, &mut takeoff)? <= threshold_m)
    }

    fn landing_start(&mut self) -> Result {
        self.set_stage(Stage::Landing);
        self.request(Request::Land, 0.0)
    }

    fn rtl_start(&mut self) -> Result {
        self.set_stage(Stage::Rtl);
        self.request(Request::Rtl, 0.0)
    }

    fn loiter_start(&mut self) -> Result {
        self.set_stage(Stage::Loiter);
        self.request(Request::Loiter, 0.0)
    }

    fn landed_start(&mut self) -> Result {
        self.set_stage(Stage::Landed);
        Ok(())
    }

    fn error_start(&mut self) -> Result {
        self.set_stage(Stage::Error);
        Ok(())
    }

    fn landing_run(&mut self) -> Result {
        let st = self.status()?;
        if st.complete {
            return self.landed_start();
        }
        Ok(())
    }

    fn landed_run(&mut self) -> Result {
        if self.c.arming().is_armed()? {
            self.c.arming().disarm()?;
        }
        if self.has_start_time() {
            let elapsed = self.elapsed_usec()?;
            let until = -timing::elapsed_sec(elapsed);
            if until > 10.0 && until <= 43200.0 {
                self.initialization_start()?;
            }
        }
        Ok(())
    }

    fn error_run(&mut self) -> Result {
        if self.c.arming().is_armed()? {
            self.c.arming().disarm()?;
        }
        Ok(())
    }

    fn testing_lights_run(&mut self) -> Result {
        self.error_run()?;
        if self.auth() == AUTH_REVOKED {
            return self.initialization_start();
        }
        let elapsed = self.elapsed_usec()?;
        let landing = self.show.as_ref().map_or(0.0, |s| s.landing_time_sec);
        if timing::elapsed_sec(elapsed) > landing {
            self.landed_start()?;
        }
        Ok(())
    }

    fn run_stage(&mut self) -> Result {
        self.update_preflight()?;
        let st = vehicle::status(self.c, &self.mode, self.request_gen)?;
        if st.owner_lost {
            return Ok(());
        }
        if st.takeoff_event && vehicle::ack_takeoff_event(self.c, &self.mode)? {
            // GCS test takeoff: the show takeoff checks apply, then climb to
            // SHOW_TAKEOFF_ALT and loiter, as the fork's do_user_takeoff_start
            self.takeoff_start()?;
            if self.stage == Stage::Takeoff {
                self.next_after_takeoff = Stage::Loiter;
            }
            return Ok(());
        }
        if st.applied && !st.init_ok {
            // the host keeps the previous controller when a request fails
            match self.stage {
                Stage::Performing | Stage::Rtl | Stage::Loiter => {
                    text(self.c, MavSeverity::CRITICAL, "SkyBrush: controller failed, landing");
                    return self.landing_start();
                }
                Stage::Takeoff => {
                    text(self.c, MavSeverity::CRITICAL, "SkyBrush: takeoff failed");
                    return self.error_start();
                }
                _ => {}
            }
        }
        match self.stage {
            Stage::Off | Stage::Init => self.initialization_start(),
            Stage::WaitForStartTime => self.wait_run(),
            Stage::Takeoff => self.takeoff_run(),
            Stage::Performing => self.performing_run(),
            Stage::Rtl | Stage::Landing => self.landing_run(),
            Stage::Loiter => Ok(()),
            Stage::Landed => self.landed_run(),
            Stage::Error => self.error_run(),
            Stage::TestingLights => self.testing_lights_run(),
        }
    }

    /// Called once when entering the mode, and on module start if already in it.
    fn mode_entered(&mut self) -> Result {
        self.clear_rc_start()?;
        self.led.configure(self.c, &self.params)?;
        self.update_pyro_instance()?;
        self.set_stage(Stage::Off);
        self.initialization_start()
    }

    // ---- fences ----

    fn fence_checks(&mut self) -> Result {
        let armed = self.c.arming().is_armed()?;
        let now = now_ms(self.c);
        if !armed {
            return Ok(());
        }
        let breaches = self.c.fence().get_breaches()?;
        let hp = HardFenceParams {
            enabled: self.params.get_i32(P::HfenceEn) != 0,
            distance: self.params.get(P::HfenceDist),
            timeout: self.params.get(P::HfenceTo),
        };
        let here = self.c.ahrs().get_location()?;
        let dist = match (&here, self.hard_breach_point.as_mut(), self.hard.state) {
            (_, _, fences::Breach::None) => 0.0,
            (Some(h), Some(bp), _) => {
                let mut h = *h;
                h.get_distance(self.c, bp)?
            }
            (Some(_), None, _) => 0.0,
            (None, _, _) => f32::MAX,
        };
        let mut need_point = false;
        let mut msg = HardMsg::None;
        let disarm = self.hard.update(&hp, breaches, now, dist, &mut need_point, &mut msg);
        if need_point {
            self.hard_breach_point = here;
        }
        if self.hard.state == fences::Breach::None {
            self.hard_breach_point = None;
        }
        match msg {
            HardMsg::CountdownStarted => text(self.c, MavSeverity::NOTICE, "Hard fence breached, countdown started"),
            HardMsg::Breached => text(self.c, MavSeverity::NOTICE, "Hard fence breached"),
            HardMsg::Resolved => text(self.c, MavSeverity::INFO, "Hard fence breach resolved"),
            HardMsg::None => {}
        }
        if disarm {
            if self.hard_text.allow(now) {
                text(self.c, MavSeverity::CRITICAL, "Hard fence breached, disarming");
            }
            vehicle::force_disarm(self.c, vehicle::DISARM_FENCEBREACH)?;
        }

        if self.stage != Stage::Performing || !self.in_mode {
            return Ok(());
        }
        let d = self.distance_from_setpoint()?;
        let bp = BubbleParams {
            enabled: self.params.get_i32(P::BfenceEn) != 0,
            dxy: self.params.get(P::BfenceDxy),
            dz: self.params.get(P::BfenceDz),
            timeout: self.params.get(P::BfenceTo),
            action: self.params.get_i32(P::BfenceAct).clamp(0, 255) as u8,
        };
        match self.bubble.update(&bp, d, now) {
            fences::BUBBLE_REPORT => self.bubble_text(None),
            fences::BUBBLE_RTL => {
                if vehicle::set_mode_reason(self.c, MODE_RTL, vehicle::REASON_FENCE_BREACHED)? {
                    self.bubble_text(Some("return to launch"));
                } else if vehicle::set_mode_reason(self.c, MODE_LAND, vehicle::REASON_FENCE_BREACHED)? {
                    self.bubble_text(Some("landing"));
                } else {
                    self.bubble_text(Some("RTL and land refused"));
                }
            }
            fences::BUBBLE_LAND => {
                if vehicle::set_mode_reason(self.c, MODE_LAND, vehicle::REASON_FENCE_BREACHED)? {
                    self.bubble_text(Some("landing"));
                } else {
                    self.bubble_text(Some("land refused"));
                }
            }
            fences::BUBBLE_DISARM => {
                self.bubble_text(Some("disarming"));
                vehicle::force_disarm(self.c, vehicle::DISARM_FENCEBREACH)?;
            }
            _ => {}
        }
        Ok(())
    }

    fn bubble_text(&mut self, what: Option<&str>) {
        if !self.bubble_text.allow(now_ms(self.c)) {
            return;
        }
        let mut buf = [0u8; 64];
        let base = b"Bubble fence breached";
        let mut n = base.len();
        buf[..n].copy_from_slice(base);
        if let Some(w) = what {
            buf[n..n + 2].copy_from_slice(b", ");
            n += 2;
            buf[n..n + w.len()].copy_from_slice(w.as_bytes());
            n += w.len();
        }
        if let Ok(s) = core::str::from_utf8(&buf[..n]) {
            text(self.c, MavSeverity::CRITICAL, s);
        }
    }

    /// Deviation from the last setpoint, NEU metres.
    fn distance_from_setpoint(&self) -> Result<[f32; 3]> {
        if self.stage != Stage::Performing {
            return Ok([0.0; 3]);
        }
        let (Some(sp), Some(here)) = (self.last_setpoint_ned, self.c.ahrs().get_relative_position_ned_origin()?) else {
            return Ok([0.0; 3]);
        };
        Ok([here.x - sp.x, here.y - sp.y, -(here.z - sp.z)])
    }

    // ---- telemetry and logs ----

    fn colour(&mut self) -> [u8; 3] {
        let c = self.lights.last_color;
        [c.r, c.g, c.b]
    }

    // ---- lights ----

    /// AP_Notify flags in the bit order used by the light logic.
    fn notify_flags(&self) -> Result<u32> {
        use ap_rust_sdk::NotifyFlags as N;
        let f = self.c.notify().get_flags()?;
        let map = [
            (N::ARMED, lo::NTF_ARMED),
            (N::FLYING, lo::NTF_FLYING),
            (N::EKF_BAD, lo::NTF_EKF_BAD),
            (N::FAILSAFE_BATTERY, lo::NTF_FAILSAFE_BATTERY),
            (N::FAILSAFE_GCS, lo::NTF_FAILSAFE_GCS),
            (N::FAILSAFE_RADIO, lo::NTF_FAILSAFE_RADIO),
            (N::COMPASS_CAL_RUNNING, lo::NTF_COMPASS_CAL_RUNNING),
            (N::INITIALISING, lo::NTF_INITIALISING),
            (N::PRE_ARM_CHECK, lo::NTF_PRE_ARM_CHECK),
            (N::PRE_ARM_GPS_CHECK, lo::NTF_PRE_ARM_GPS_CHECK),
        ];
        Ok(map.iter().filter(|(a, _)| f & a != 0).fold(0, |acc, (_, b)| acc | b))
    }

    fn lights_gps_ms(&self) -> Result<u32> {
        match self.gps_epoch_usec()? {
            0 => Ok(now_ms(self.c)),
            usec => Ok((usec / 1000) as u32),
        }
    }

    fn update_lights(&mut self) -> Result {
        let c = self.c;
        let elapsed_sec = timing::elapsed_sec(self.elapsed_usec()?);
        // as the fork, advance the controller at the light rate so lights-only
        // runs play and colours do not wait for the next setpoint
        let show_colour = match self.show.as_mut() {
            Some(s) if elapsed_sec >= 0.0 => match s.controller.update_time_msec(secs_to_msec(elapsed_sec)) {
                Ok(()) => s.controller.output().color,
                Err(_) => None,
            },
            _ => None,
        };
        let i = LightInputs {
            stage: self.stage as u8,
            mode: c.vehicle().get_mode()?,
            notify: self.notify_flags()?,
            spooling: c.motors().get_spool_state()? != 0,
            bubble_flash: self.bubble.state != fences::Breach::None
                && self.params.get_i32(P::BfenceAct) == fences::BUBBLE_FLASH as i32,
            auth: self.auth(),
            preflight_failures: self.preflight_failures,
            performance_completed: self.performance_completed(),
            start_scheduled: self.has_start_time(),
            time_to_takeoff_sec: self.time_until_takeoff_sec()?,
            elapsed_sec,
            show_colour,
            gps_ms: self.lights_gps_ms()?,
            now_ms: now_ms(c),
            pre_lights: self.params.get_i32(P::PreLights),
            options: self.params.get_i32(P::Options),
        };
        let frame = self.lights.update(&i);
        self.led.write(c, frame, &self.params)
    }

    /// Compass calibration result flashes and RTCM rate sampling
    fn check_events(&mut self) -> Result {
        let c = self.c;
        let now = now_ms(c);
        match c.notify().get_event_counts() {
            Ok(counts) => {
                let r = self.cal.update(counts);
                if r.failed {
                    self.lights.flash_failure(now);
                } else if r.saved {
                    self.lights.flash_success(now);
                }
            }
            Err(ap_rust_sdk::Error::Unavailable) => {}
            Err(e) => return Err(e),
        }
        // the status reports MAVLink channels 1 and 2 (TELEM1 and TELEM2)
        for (i, rate) in self.rtcm.iter_mut().enumerate() {
            match c.gps().rtcm_message_count(i as u8 + 1) {
                Ok(total) => rate.update(total, now),
                Err(ap_rust_sdk::Error::Unavailable) => {}
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    pub fn handle_led_control(&mut self, payload: &[u8]) -> bool {
        let group = self.params.get_i32(P::Group).clamp(0, 255) as u8;
        self.lights.handle_led_control(payload, payload[0], group, now_ms(self.c))
    }

    /// MAV_CMD_USER_1 param1=4; returns a MAV_RESULT.
    pub fn handle_light_source(&mut self, param2: f32, param3: f32, x: i32) -> u8 {
        self.lights.handle_source_command(param2, param3, x)
    }

    pub fn handle_tunnel(&mut self, payload: &[u8]) -> bool {
        if mav::u16_at(payload, 0) != mav::TUNNEL_PIXEL_GRID {
            return false;
        }
        let len = (payload[4] as usize).min(128);
        self.lights.pixel.handle_update(&payload[5..5 + len])
    }

    fn status_basic(&mut self) -> Result<telemetry::Basic> {
        let elapsed = self.elapsed_usec()?;
        let gps = self.c.gps();
        let i = gps.primary_sensor()?;
        let rgb = self.colour();
        self.last_rgb = rgb;
        let rgb565 = ((rgb[0] as u16 >> 3) << 11) | ((rgb[1] as u16 >> 2) << 5) | (rgb[2] as u16 >> 3);
        let elapsed_sec = match (&self.show, self.stage) {
            (Some(s), Stage::Performing) if s.controller.is_output_valid() => {
                s.controller.output_time().warped_time_in_scene_sec
            }
            _ => timing::elapsed_sec(elapsed),
        };
        let off_position = {
            let d = self.distance_from_setpoint()?;
            hypotf(d[0], d[1]) > self.params.get(P::MaxXyErr) || d[2].abs() > self.params.get(P::MaxZErr)
        };
        Ok(telemetry::Basic {
            start_time: timing::encode_start_time(self.params.get_i32(P::StartTime), self.params.get_i32(P::StartMsec)),
            led_rgb565: rgb565,
            show_ok: self.show_ok(),
            start_scheduled: self.has_start_time(),
            origin_set: self.origin_set(),
            orientation_set: self.orientation_set(),
            fence_enabled: self.c.fence().get_enabled_fences()? != 0,
            authorized: self.auth() != AUTH_REVOKED,
            gps_time_bad: self.clock.use_gps && !self.gps_time_ok()?,
            fence_breached: self.c.fence().get_breaches()? != 0,
            preflight_failures: self.preflight_failures,
            stage: self.stage as u8,
            gps_status: gps_value(gps.status(i))?,
            num_sats: gps_value(gps.num_sats(i))?,
            boot_count: self.boot_count,
            auth_scope: self.auth(),
            esc_error: self.esc.high(self.params.get(P::MaxEscErr)),
            off_position,
            elapsed_sec,
            rtcm_counts: [self.rtcm[0].count(now_ms(self.c)), self.rtcm[1].count(now_ms(self.c))],
        })
    }

    fn send_status(&mut self) -> Result {
        let now = now_ms(self.c);
        let compact = self.params.get_i32(P::TelProfile) == 1;
        let period = if compact { 1000 } else { 500 };
        if self.last_status_ms != 0 && now.wrapping_sub(self.last_status_ms) < period {
            return Ok(());
        }
        self.last_status_ms = now;
        if self.params.get(P::MaxEscErr) >= 0.0 {
            crate::report(self.c, "esc errors", self.esc.update(self.c))?;
        }
        let basic = crate::report(self.c, "status_basic", self.status_basic())?;
        let mut data = [0u8; 48];
        let (msgid, payload): (u32, &[u8]) = if compact {
            let ext = self.extended()?;
            telemetry::encode_extended(&basic, &ext, &mut data);
            (mav::MSG_DATA96, &data[..telemetry::EXTENDED_LEN])
        } else {
            telemetry::encode_basic(&basic, &mut data);
            (mav::MSG_DATA16, &data[..telemetry::BASIC_LEN])
        };
        let mut buf = [0u8; 98];
        buf[0] = mav::TYPE_STATUS;
        buf[1] = payload.len() as u8;
        buf[2..2 + payload.len()].copy_from_slice(payload);
        let size = if msgid == mav::MSG_DATA96 { 98 } else { 18 };
        for chan in 0..8u8 {
            match crate::report(self.c, "send_chan", self.c.mavlink().send_chan(chan, msgid, &buf[..size], None))? {
                None => break,
                Some(_) => {}
            }
        }
        Ok(())
    }

    fn extended(&self) -> Result<telemetry::Extended> {
        let ahrs = self.c.ahrs();
        let mut e = telemetry::Extended::default();
        if let Some(loc) = ahrs.get_location()? {
            e.lat = loc.lat();
            e.lng = loc.lng();
            let mut l = loc;
            e.alt_mm = l.get_alt_m(self.c, crate::coords::ALT_ABSOLUTE)?.map_or(0, |a| (a * 1000.0) as i32);
        }
        e.relative_alt_mm = (-ahrs.get_relative_position_d_home()? * 1000.0) as i32;
        if let Some(v) = ahrs.get_velocity_ned()? {
            e.vel_cms = [(v.x * 100.0) as i16, (v.y * 100.0) as i16, (v.z * 100.0) as i16];
        }
        let yaw = ahrs.get_yaw_rad()?.to_degrees();
        e.hdg_cd = ((if yaw < 0.0 { yaw + 360.0 } else { yaw }) * 100.0) as u16;
        let gps = self.c.gps();
        e.hdop = gps_value(gps.get_hdop(0))?;
        e.vdop = gps_value(gps.get_vdop(0))?;
        Ok(e)
    }

    fn write_logs(&mut self) -> Result {
        let elapsed = self.elapsed_usec()?;
        let d = self.distance_from_setpoint()?;
        let rgb = self.last_rgb;
        let (pos, scene, scene_ms) = match &self.show {
            Some(s) if s.controller.is_output_valid() => {
                let t = s.controller.output_time();
                let p = s.controller.output().position.unwrap_or(Vec3::ZERO);
                (
                    [p.x / 1000.0, p.y / 1000.0, p.z / 1000.0],
                    t.scene.map_or(0, |v| v as u8),
                    (t.warped_time_in_scene_sec * 1000.0) as i32,
                )
            }
            _ => ([0.0; 3], 0, 0),
        };
        // sample the clock last so ClockMS is close to the logger's TimeUS
        let clock_ms = timing::elapsed_msec(if self.has_start_time() { self.elapsed_usec()? } else { elapsed });
        logging::show(
            self.c,
            &logging::ShowRow {
                clock_ms,
                stage: self.stage as u8,
                scene,
                scene_ms,
                pos_m: pos,
                rgb,
                hdist: hypotf(d[0], d[1]),
                vdist: d[2],
            },
        )?;
        let now = now_ms(self.c);
        if now.wrapping_sub(self.last_fence_log_ms) >= 1000 {
            self.last_fence_log_ms = now;
            logging::fence(
                self.c,
                &logging::FenceRow {
                    geo_enabled: self.c.fence().get_enabled_fences()?,
                    geo_breaches: self.c.fence().get_breaches()?,
                    geo_count: vehicle::fence_breach_count(self.c)?,
                    hard: self.hard.state as u8,
                    bubble: self.bubble.state as u8,
                    bubble_count: self.bubble.action_counter,
                },
            )?;
        }
        Ok(())
    }

    // ---- main tick, 50 Hz ----

    pub fn tick(&mut self) -> Result {
        let c = self.c;
        self.tick = self.tick.wrapping_add(1);
        crate::report(c, "keepalive", vehicle::keepalive(c, &self.mode))?;
        let even = self.tick % 2 == 0;
        if even {
            crate::report(c, "params", self.param_changes())?;
            self.apply_pending_start_time()?;
            if self.tick % 50 == 0 {
                crate::report(c, "show mode", self.switch_to_show_mode())?;
            }
            crate::report(c, "rc switch", self.check_rc_start_switch())?;
            let ekf = value(&self.vparams.fs_ekf_action).unwrap_or(0.0) as i32 == 42;
            if ekf != self.ekf_disarm {
                self.ekf_disarm = ekf;
                vehicle::set_ekf_disarm(c, &self.mode, ekf)?;
            }
        }
        let armed = crate::report(c, "is_armed", c.arming().is_armed())?;
        crate::report(c, "yaw", self.note_armed(armed))?;
        let in_mode = crate::report(c, "get_mode", c.vehicle().get_mode())? == MODE_NUMBER;
        if in_mode && !self.in_mode {
            self.in_mode = true;
            crate::report(c, "mode_entered", self.mode_entered())?;
        } else if !in_mode && self.in_mode {
            self.in_mode = false;
            self.set_stage(Stage::Off);
            self.clear_rc_start()?;
            crate::report(c, "led", self.led.configure(c, &self.params))?;
            crate::report(c, "pyro", self.update_pyro_instance())?;
        }
        if self.in_mode {
            crate::report(c, "stage", self.run_stage())?;
        }
        if even {
            crate::report(c, "fences", self.fence_checks())?;
            crate::report(c, "pyro", self.update_pyro())?;
            crate::report(c, "events", self.check_events())?;
            crate::report(c, "lights", self.update_lights())?;
            crate::report(c, "logs", self.write_logs())?;
        } else {
            crate::report(c, "led", self.led.repeat(c))?;
        }
        crate::report(c, "status", self.send_status())
    }
}

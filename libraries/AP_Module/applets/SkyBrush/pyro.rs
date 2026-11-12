// SPDX-License-Identifier: GPL-3.0-or-later
//! Pyro: show event firing, safety gate, auto-off, output devices selected by
//! SHOW_PYRO_TYPE and the MAV_CMD_USER_1 pyro test (the fork's
//! AC_DroneShowManager_Pyro/Events and DroneShowPyroDevice_*).
//!
//! Auto-off at SHOW_PYRO_ITIME is done here. Servo devices also get a
//! set_output_pwm_chan_timeout override as a backstop, which reverts to an
//! explicit off baseline (function 105 scaled to SERVOn_MIN) if the module
//! stops. Relay outputs have no such backstop. Cobra times out on its own.

use ap_rust_sdk::enums::{MavSeverity, SrvChannelFunction};
use ap_rust_sdk::{Context, NotifyFlags as N, Result, SerialFlowControl, SerialPort};

use super::{now_ms, text, App, Stage};
use crate::{
    logging, mavlink as mav,
    params::P,
    show::events::{Event, EVENT_TYPE_LIGHT, EVENT_TYPE_PYRO, LIGHT_SET_PIXEL_X, LIGHT_SET_PIXEL_Y, LIGHT_SET_SOURCE, PYRO_OFF},
    support,
};

pub const RESULT_SUCCESS: u8 = 0;
pub const RESULT_FAILURE: u8 = 1;
pub const RESULT_NOT_SUPPORTED: u8 = 2;
pub const RESULT_UNSAFE: u8 = 3;
pub const RESULT_TIME_MISSED: u8 = 4;

const EVENT_TIMING_TOLERANCE_SEC: f32 = 3.0;
const MAX_EVENTS_PER_TICK: u8 = 10;
// SERVOn_FUNCTION 105 = scripting12
const FN_PYRO: SrvChannelFunction = SrvChannelFunction(105);
const MAX_CHANNELS: usize = 8;
const MAX_SERVOS: u8 = 32;
const MAX_RELAYS: u8 = 16;
const SCRIPTING_SERIAL_LED_TYPE: i32 = 9;
const COBRA_FIRE_CUES: u8 = 0x46;
const BACKSTOP_REFRESH_MS: u32 = 1000;

#[derive(Clone, Copy)]
struct Servo {
    chan: u8,
    min: u16,
    max: u16,
}

enum Device {
    Debug,
    // one output, logical channel 0 = 85%, 1 = 100%
    SingleServo(Servo),
    MultipleServos { servos: [Servo; MAX_CHANNELS], n: u8 },
    Cobra(SerialPort),
    Relay { idx: [u8; MAX_CHANNELS], n: u8 },
}

#[derive(Default)]
struct TestState {
    last_fired_ms: u32,
    next_channel: u8,
    remaining: u8,
    delta_ms: u32,
}

pub struct Pyro {
    device: Option<Device>,
    kind: i32,
    // ms when each channel was fired, 0 when off
    ignited_at: [u32; MAX_CHANNELS],
    // ms of the last servo backstop refresh per channel
    backstop_ms: [u32; MAX_CHANNELS],
    // logical channel currently driving the single servo output
    single_owner: Option<u8>,
    // function 105 scaled value that gives SERVOn_MIN
    baseline: f32,
    test: TestState,
}

impl Default for Pyro {
    fn default() -> Self {
        Self {
            device: None,
            kind: -1,
            ignited_at: [0; MAX_CHANNELS],
            backstop_ms: [0; MAX_CHANNELS],
            single_owner: None,
            baseline: -4500.0,
            test: TestState::default(),
        }
    }
}

fn param(c: &Context<'_>, name: &str) -> Result<Option<f32>> {
    match c.parameter(name)? {
        Some(p) => p.get(),
        None => Ok(None),
    }
}

fn indexed_param(c: &Context<'_>, fmt: core::fmt::Arguments<'_>) -> Result<Option<f32>> {
    let mut buf = [0u8; 16];
    let name = support::fmt(&mut buf, fmt);
    param(c, name)
}

fn servo(c: &Context<'_>, chan: u8) -> Result<Servo> {
    let n = chan + 1;
    Ok(Servo {
        chan,
        min: indexed_param(c, format_args!("SERVO{}_MIN", n))?.unwrap_or(1100.0) as u16,
        max: indexed_param(c, format_args!("SERVO{}_MAX", n))?.unwrap_or(1900.0) as u16,
    })
}

fn duty(s: &Servo, pct: u32) -> u16 {
    let span = s.max.saturating_sub(s.min) as u32;
    s.min + (span * pct.min(100) / 100) as u16
}

impl Device {
    fn num_channels(&self) -> u8 {
        match self {
            Device::Debug => 4,
            Device::SingleServo(_) => 2,
            Device::MultipleServos { n, .. } => *n,
            Device::Cobra(_) => 6,
            Device::Relay { n, .. } => *n,
        }
    }

    fn servo_for(&self, ch: u8) -> Option<Servo> {
        match self {
            Device::SingleServo(s) => Some(*s),
            Device::MultipleServos { servos, .. } => Some(servos[ch as usize]),
            _ => None,
        }
    }
}

impl Pyro {
    fn new_device(&mut self, c: &Context<'_>, kind: i32, led_type: i32) -> Result<Option<Device>> {
        let mut servos = [Servo { chan: 0, min: 0, max: 0 }; MAX_CHANNELS];
        let mut n = 0u8;
        let dev = match kind {
            1 => Some(Device::Debug),
            2 => match c.srv_channels().find_channel(FN_PYRO)? {
                Some(ch) => {
                    servos[0] = servo(c, ch)?;
                    n = 1;
                    Some(Device::SingleServo(servos[0]))
                }
                None => None,
            },
            3 => {
                for ch in 0..MAX_SERVOS {
                    if n as usize == MAX_CHANNELS {
                        break;
                    }
                    match indexed_param(c, format_args!("SERVO{}_FUNCTION", ch + 1))? {
                        Some(f) if f as i32 == FN_PYRO.0 => {
                            servos[n as usize] = servo(c, ch)?;
                            n += 1;
                        }
                        Some(_) => {}
                        None => break,
                    }
                }
                (n > 0).then_some(Device::MultipleServos { servos, n })
            }
            4 => {
                // a scripting UART may already drive SHOW_LED0_TYPE 9
                let instance = if led_type == SCRIPTING_SERIAL_LED_TYPE { 1 } else { 0 };
                match c.serial().find_serial(instance)? {
                    Some(port) => {
                        port.set_flow_control(c, SerialFlowControl::Disabled)?;
                        // configured SERIALx_BAUD; Cobra 6M uses 115200
                        port.begin(c, None)?;
                        Some(Device::Cobra(port))
                    }
                    None => {
                        text(c, MavSeverity::ERROR, "Cobra: set SERIALx_PROTOCOL to Scripting (28)");
                        None
                    }
                }
            }
            5 => {
                // relays with RELAYn_FUNCTION = 1 (relay)
                let mut idx = [0u8; MAX_CHANNELS];
                for r in 0..MAX_RELAYS {
                    if n as usize == MAX_CHANNELS {
                        break;
                    }
                    match indexed_param(c, format_args!("RELAY{}_FUNCTION", r + 1))? {
                        Some(f) if f as i32 == 1 => {
                            idx[n as usize] = r;
                            n += 1;
                        }
                        Some(_) => {}
                        None => break,
                    }
                }
                (n > 0).then_some(Device::Relay { idx, n })
            }
            _ => None,
        };
        if n > 0 && matches!(dev, Some(Device::SingleServo(_)) | Some(Device::MultipleServos { .. })) {
            // one scaled baseline drives every pyro output, so it is only the
            // off state if all of them share the same reversal
            let Some(rev) = Self::common_reversal(c)? else {
                text(c, MavSeverity::ERROR, "Pyro servos have mixed SERVOn_REVERSED");
                return Ok(None);
            };
            self.baseline = if rev { 4500.0 } else { -4500.0 };
        }
        Ok(dev)
    }

    /// Reversal shared by all pyro function outputs, None if they differ.
    fn common_reversal(c: &Context<'_>) -> Result<Option<bool>> {
        let mut common = None;
        for ch in 0..MAX_SERVOS {
            match indexed_param(c, format_args!("SERVO{}_FUNCTION", ch + 1))? {
                Some(f) if f as i32 == FN_PYRO.0 => {
                    let rev = indexed_param(c, format_args!("SERVO{}_REVERSED", ch + 1))?.unwrap_or(0.0) != 0.0;
                    if common.is_some_and(|r| r != rev) {
                        return Ok(None);
                    }
                    common = Some(rev);
                }
                Some(_) => {}
                None => break,
            }
        }
        Ok(Some(common.unwrap_or(false)))
    }

    /// Rebuild the device if SHOW_PYRO_TYPE changed; outputs of the old one are turned off.
    pub fn update_instance(&mut self, c: &Context<'_>, kind: i32, led_type: i32) -> Result {
        if kind == self.kind {
            return Ok(());
        }
        self.deinit(c)?;
        self.kind = kind;
        self.device = self.new_device(c, kind, led_type)?;
        // establish the off state, as the fork's init_impl()
        self.all_off(c)
    }

    fn all_off(&mut self, c: &Context<'_>) -> Result {
        let physical = matches!(
            self.device,
            Some(Device::SingleServo(_)) | Some(Device::MultipleServos { .. }) | Some(Device::Relay { .. })
        );
        if physical {
            for ch in 0..self.num_channels() {
                self.output_off(c, ch)?;
            }
        }
        Ok(())
    }

    fn deinit(&mut self, c: &Context<'_>) -> Result {
        self.all_off(c)?;
        self.device = None;
        self.ignited_at = [0; MAX_CHANNELS];
        self.single_owner = None;
        self.test = TestState::default();
        Ok(())
    }

    fn servo_on(c: &Context<'_>, baseline: f32, s: &Servo, pwm: u16, timeout_ms: u16) -> Result {
        let srv = c.srv_channels();
        // keeps have_pwm clear, so the override reverts to the baseline
        srv.set_output_scaled(FN_PYRO, baseline)?;
        srv.set_output_pwm_chan_timeout(s.chan, pwm, timeout_ms)
    }

    fn servo_off(c: &Context<'_>, baseline: f32, s: &Servo) -> Result {
        let srv = c.srv_channels();
        srv.set_output_scaled(FN_PYRO, baseline)?;
        // zero timeout ends the override; the output falls back to the baseline
        srv.set_output_pwm_chan_timeout(s.chan, s.min, 0)
    }

    fn fire_pwm(&self, ch: u8, s: &Servo) -> u16 {
        match self.device {
            Some(Device::SingleServo(_)) => duty(s, if ch == 0 { 85 } else { 100 }),
            _ => s.max,
        }
    }

    /// Host timeout for an output, at most the 65535 ms the host accepts; it is
    /// renewed every second so longer ignitions stay on.
    fn backstop_ms(itime: i32, elapsed_ms: u32) -> u16 {
        if itime > 0 {
            (itime as u32).saturating_sub(elapsed_ms).clamp(1, u16::MAX as u32) as u16
        } else {
            u16::MAX
        }
    }

    fn output_on(&mut self, c: &Context<'_>, ch: u8, itime: i32) -> Result<u8> {
        let Some(d) = &self.device else {
            return Ok(RESULT_NOT_SUPPORTED);
        };
        match d {
            Device::Debug => {
                let mut buf = [0u8; 24];
                text(c, MavSeverity::INFO, support::fmt(&mut buf, format_args!("Firing channel {}", ch)));
            }
            Device::SingleServo(_) | Device::MultipleServos { .. } => {
                let s = d.servo_for(ch).unwrap();
                let pwm = self.fire_pwm(ch, &s);
                Self::servo_on(c, self.baseline, &s, pwm, Self::backstop_ms(itime, 0))?;
                if matches!(self.device, Some(Device::SingleServo(_))) {
                    self.single_owner = Some(ch);
                }
            }
            Device::Cobra(port) => {
                // channel 1 is the MSB: 123456xx
                let mask = (0x80u8 >> ch) & 0xFC;
                let frame = [COBRA_FIRE_CUES, mask, COBRA_FIRE_CUES.wrapping_add(mask), 0x0D];
                if port.write(c, &frame)? != frame.len() {
                    return Ok(RESULT_FAILURE);
                }
            }
            // the host turns the relay off if the module stops renewing it
            Device::Relay { idx, .. } => c.relay().on_for_ms(idx[ch as usize], Self::backstop_ms(itime, 0))?,
        }
        Ok(RESULT_SUCCESS)
    }

    fn output_off(&mut self, c: &Context<'_>, ch: u8) -> Result<u8> {
        let Some(d) = &self.device else {
            return Ok(RESULT_NOT_SUPPORTED);
        };
        match d {
            Device::Debug => {
                let mut buf = [0u8; 32];
                text(c, MavSeverity::INFO, support::fmt(&mut buf, format_args!("Turning off channel {}", ch)));
            }
            Device::SingleServo(_) | Device::MultipleServos { .. } => {
                let s = d.servo_for(ch).unwrap();
                let single = matches!(self.device, Some(Device::SingleServo(_)));
                // the shared output belongs to the last fired channel; turning
                // off the other one must not cut it (the fork does)
                if !single || self.single_owner.is_none_or(|o| o == ch) {
                    Self::servo_off(c, self.baseline, &s)?;
                    if single {
                        self.single_owner = None;
                    }
                }
            }
            // Cobra 6M ends the ignition on its own
            Device::Cobra(_) => {}
            Device::Relay { idx, .. } => c.relay().off(idx[ch as usize])?,
        }
        Ok(RESULT_SUCCESS)
    }

    fn num_channels(&self) -> u8 {
        self.device.as_ref().map_or(0, |d| d.num_channels())
    }

    /// Fire a channel without safety checks; records the ignition time.
    pub fn fire(&mut self, c: &Context<'_>, ch: u8, itime: i32) -> Result<u8> {
        if ch >= self.num_channels() {
            return Ok(RESULT_NOT_SUPPORTED);
        }
        let r = self.output_on(c, ch, itime)?;
        if r == RESULT_SUCCESS {
            let now = now_ms(c).max(1);
            self.ignited_at[ch as usize] = now;
            self.backstop_ms[ch as usize] = now;
        }
        Ok(r)
    }

    pub fn off(&mut self, c: &Context<'_>, ch: u8) -> Result<u8> {
        if ch >= self.num_channels() {
            return Ok(RESULT_NOT_SUPPORTED);
        }
        // cleared even on failure so off is not retried forever, as the fork
        self.ignited_at[ch as usize] = 0;
        self.output_off(c, ch)
    }

    /// Auto-off after SHOW_PYRO_ITIME (<= 0: never) and servo backstop refresh.
    fn turn_off_if_needed(&mut self, c: &Context<'_>, itime: i32) -> Result {
        let now = now_ms(c);
        for ch in 0..self.num_channels() {
            let at = self.ignited_at[ch as usize];
            if at == 0 {
                continue;
            }
            let elapsed = now.wrapping_sub(at);
            if itime > 0 && elapsed >= itime as u32 {
                self.off(c, ch)?;
                continue;
            }
            // renew the host timeout so a stalled module cannot leave the
            // output on, and outputs last longer than the 65 s timeout limit
            if now.wrapping_sub(self.backstop_ms[ch as usize]) >= BACKSTOP_REFRESH_MS {
                self.backstop_ms[ch as usize] = now;
                let s = match &self.device {
                    Some(Device::SingleServo(s)) if self.single_owner == Some(ch) => *s,
                    Some(Device::MultipleServos { servos, .. }) => servos[ch as usize],
                    Some(Device::Relay { idx, .. }) => {
                        c.relay().on_for_ms(idx[ch as usize], Self::backstop_ms(itime, elapsed))?;
                        continue;
                    }
                    _ => continue,
                };
                let pwm = self.fire_pwm(ch, &s);
                Self::servo_on(c, self.baseline, &s, pwm, Self::backstop_ms(itime, elapsed))?;
            }
        }
        Ok(())
    }

    fn update_test(&mut self, c: &Context<'_>, armed: bool, itime: i32) -> Result {
        let t = &mut self.test;
        if t.last_fired_ms == 0 || t.remaining == 0 {
            return Ok(());
        }
        if armed {
            self.test = TestState::default();
            text(c, MavSeverity::WARNING, "Pyro test aborted, vehicle armed");
            return Ok(());
        }
        let now = now_ms(c);
        if now.wrapping_sub(t.last_fired_ms) < t.delta_ms {
            return Ok(());
        }
        t.last_fired_ms = now.max(1);
        let ch = t.next_channel;
        t.remaining -= 1;
        t.next_channel = t.next_channel.wrapping_add(1);
        if t.remaining == 0 {
            text(c, MavSeverity::INFO, "Finished pyro test");
            self.test = TestState::default();
        }
        if self.fire(c, ch, itime)? != RESULT_SUCCESS {
            // stop on the first failure
            self.test = TestState::default();
        }
        Ok(())
    }
}

impl App<'_, '_> {
    pub(super) fn update_pyro_instance(&mut self) -> Result {
        let kind = self.params.get_i32(P::PyroType);
        let led = self.params.get_i32(P::Led0Type);
        self.pyro.update_instance(self.c, kind, led)
    }

    /// The fork's _is_pyro_safe_to_fire().
    fn pyro_safe_to_fire(&self) -> Result<bool> {
        if self.stage != Stage::Performing {
            return Ok(false);
        }
        let f = self.c.notify().get_flags()?;
        let need = N::ARMED | N::FLYING | N::HAVE_POS_ABS;
        let bad = N::GPS_GLITCHING | N::EKF_BAD | N::FAILSAFE_BATTERY;
        if f & need != need || f & bad != 0 || self.c.fence().get_breaches()? != 0 {
            return Ok(false);
        }
        // at the expected position; no setpoint or position yet is unsafe
        let (Some(sp), Some(here)) = (self.last_setpoint_ned, self.c.ahrs().get_relative_position_ned_origin()?) else {
            return Ok(false);
        };
        let (dx, dy, dz) = (here.x - sp.x, here.y - sp.y, here.z - sp.z);
        let max_xy = self.params.get(P::MaxXyErr);
        let max_z = self.params.get(P::MaxZErr);
        if max_xy > 0.0 && crate::show::fmath::hypotf(dx, dy) > max_xy {
            return Ok(false);
        }
        if max_z > 0.0 && dz.abs() > max_z {
            return Ok(false);
        }
        Ok(matches!(self.altitude_above_home_m()?, Some(a) if a >= self.params.get(P::PyroMinalt)))
    }

    /// LIGHT events share the show event stream with pyro events.
    fn light_event(&mut self, e: &Event) -> u8 {
        let v = e.payload_u32();
        let ok = match e.subtype {
            LIGHT_SET_SOURCE => self.lights.configure_source_from_event(&e.payload),
            LIGHT_SET_PIXEL_X => self.lights.pixel.set_column(v),
            LIGHT_SET_PIXEL_Y => self.lights.pixel.set_row(v),
            _ => return RESULT_NOT_SUPPORTED,
        };
        if ok {
            RESULT_SUCCESS
        } else {
            RESULT_FAILURE
        }
    }

    /// The fork's _trigger_show_events(): drains due events in Performing.
    fn trigger_show_events(&mut self) -> Result {
        if self.stage != Stage::Performing {
            return Ok(());
        }
        let itime = self.params.get_i32(P::PyroItime);
        for _ in 0..MAX_EVENTS_PER_TICK {
            let Some(show) = self.show.as_mut() else {
                return Ok(());
            };
            let Some(e) = show.controller.next_event() else {
                return Ok(());
            };
            let t = show.controller.output_time();
            let (scene, scene_sec) = (t.scene.map_or(0, |v| v as u8), t.warped_time_in_scene_sec);
            let result = match e.kind {
                EVENT_TYPE_PYRO => {
                    if self.pyro.device.is_none() {
                        RESULT_NOT_SUPPORTED
                    } else if e.payload_u32() == PYRO_OFF {
                        self.pyro.off(self.c, e.subtype)?
                    } else if !self.pyro_safe_to_fire()? {
                        RESULT_UNSAFE
                    } else if (e.time_msec as f32) < (scene_sec - EVENT_TIMING_TOLERANCE_SEC) * 1000.0 {
                        RESULT_TIME_MISSED
                    } else {
                        self.pyro.fire(self.c, e.subtype, itime)?
                    }
                }
                EVENT_TYPE_LIGHT => self.light_event(&e),
                _ => RESULT_NOT_SUPPORTED,
            };
            let clock_ms = crate::timing::elapsed_msec(self.elapsed_usec()?);
            logging::event(
                self.c,
                clock_ms,
                scene,
                (scene_sec * 1000.0) as i32,
                e.kind,
                e.subtype,
                e.payload_u32(),
                result,
            )?;
        }
        Ok(())
    }

    /// 25 Hz: show events, the pyro test and auto-off.
    pub(super) fn update_pyro(&mut self) -> Result {
        self.trigger_show_events()?;
        let armed = self.c.arming().is_armed()?;
        let itime = self.params.get_i32(P::PyroItime);
        self.pyro.update_test(self.c, armed, itime)?;
        self.pyro.turn_off_if_needed(self.c, itime)
    }

    /// MAV_CMD_USER_1 param1=2: param2 start channel, param3 count (0 = all),
    /// param4 delay between channels in seconds. Refused while armed.
    pub fn handle_pyro_test(&mut self, param2: f32, param3: f32, param4: f32) -> Result<u8> {
        if self.c.arming().is_armed()? {
            return Ok(mav::MAV_RESULT_DENIED);
        }
        let start = if (0.0..255.0).contains(&param2) { param2 as u8 } else { 255 };
        let mut count = if (0.0..256.0).contains(&param3) { param3 as u8 } else { 255 };
        let delay_ms = if param4.is_finite() && param4 >= 0.0 { (param4 * 1000.0) as u32 } else { 0 };
        if count == 0 {
            count = 255;
        }
        if start == 255 {
            return Ok(mav::MAV_RESULT_UNSUPPORTED);
        }
        let num = self.pyro.num_channels();
        if self.pyro.device.is_none() || start >= num {
            return Ok(mav::MAV_RESULT_FAILED);
        }
        if start as u16 + count as u16 > num as u16 {
            count = num - start;
        }
        let now = now_ms(self.c);
        self.pyro.test = TestState {
            last_fired_ms: now.wrapping_sub(delay_ms).max(1),
            next_channel: start,
            remaining: count,
            delta_ms: delay_ms,
        };
        text(self.c, MavSeverity::INFO, "Starting pyro test");
        Ok(mav::MAV_RESULT_ACCEPTED)
    }
}

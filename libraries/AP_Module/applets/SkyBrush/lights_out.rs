// SPDX-License-Identifier: GPL-3.0-or-later
//! LED colour state machine, LED_CONTROL light signals, pixel grid and LED
//! output processing (gamma, min brightness, repeats), as the fork's
//! AC_DroneShowManager_Lights.cpp, _PixelGrid.cpp and DroneShowLED.
//! No SDK dependency, so it can be host-tested; backends are in lights_backends.rs.

use alloc::vec::Vec;

use crate::show::fmath::{cosf, powf, roundf};
use crate::show::lights::{Rgb, Rgbw, RgbwConversion};

// `LightInputs::notify` bits, mirroring AP_Notify::flags
pub const NTF_ARMED: u32 = 1 << 0;
pub const NTF_FLYING: u32 = 1 << 1;
pub const NTF_EKF_BAD: u32 = 1 << 2;
pub const NTF_FAILSAFE_BATTERY: u32 = 1 << 3;
pub const NTF_FAILSAFE_GCS: u32 = 1 << 4;
pub const NTF_FAILSAFE_RADIO: u32 = 1 << 5;
pub const NTF_COMPASS_CAL_RUNNING: u32 = 1 << 6;
pub const NTF_INITIALISING: u32 = 1 << 7;
pub const NTF_PRE_ARM_CHECK: u32 = 1 << 8;
pub const NTF_PRE_ARM_GPS_CHECK: u32 = 1 << 9;

// app::Stage discriminants
const STAGE_WAIT_FOR_START: u8 = 2;
const STAGE_PERFORMING: u8 = 4;
const STAGE_RTL: u8 = 5;
const STAGE_LOITER: u8 = 6;
const STAGE_LANDING: u8 = 7;
const STAGE_LANDED: u8 = 8;
const STAGE_ERROR: u8 = 9;
const STAGE_TESTING_LIGHTS: u8 = 10;

// Copter mode numbers, hardcoded as in the fork
const MODE_RTL: u8 = 6;
const MODE_LAND: u8 = 9;
const MODE_SMART_RTL: u8 = 21;
const MODE_SHOW: u8 = 127;

// SHOW_OPTIONS bits
const OPT_DISABLE_FAILSAFE_LIGHTS: i32 = 1 << 0;
const OPT_PIXEL_GRID_ON_GROUND: i32 = 1 << 3;

// SHOW_START_AUTH values
const AUTH_REVOKED: u8 = 0;
const AUTH_LIVE: u8 = 1;

const RED: Rgb = Rgb::new(255, 0, 0);
const YELLOW: Rgb = Rgb::new(255, 255, 0);
const GREEN: Rgb = Rgb::new(0, 255, 0);
const GREEN_DIM: Rgb = Rgb::new(0, 128, 0);
const ORANGE: Rgb = Rgb::new(255, 128, 0);
const WHITE_DIM: Rgb = Rgb::new(128, 128, 128);
const LIGHT_BLUE: Rgb = Rgb::new(0, 128, 255);
const MAGENTA: Rgb = Rgb::new(255, 0, 255);

// 8-bit masks over the 125 ms slots of a second
const ALL_ON: u8 = 0b1111_1111;
const BLINK: u8 = 0b1111_0000;
const BLINK_TWICE_PER_SECOND: u8 = 0b1100_1100;
const FLASH_ONCE_PER_SECOND: u8 = 0b1000_0000;
const FLASH_TWICE_PER_SECOND: u8 = 0b1010_0000;
const FLASH_FOUR_TIMES_PER_SECOND: u8 = 0b1010_1010;

pub const LED_CONTROL_MAGIC: u8 = 42;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Effect {
    #[default]
    Off,
    Solid,
    Blinking,
    Breathing,
}

impl Effect {
    fn from_u8(v: u8) -> Self {
        match v {
            0 => Effect::Off,
            1 => Effect::Solid,
            2 => Effect::Blinking,
            _ => Effect::Breathing,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Priority {
    #[default]
    None,
    Broadcast,
    Individual,
    Internal,
}

/// Modulation factor 0..1 of an effect at `ts` ms.
fn modulation(ts: u32, effect: Effect, period_ms: u16, phase_ms: u16) -> f32 {
    match effect {
        Effect::Off => return 0.0,
        Effect::Solid => return 1.0,
        _ if period_ms == 0 => return 1.0,
        _ => {}
    }
    let t = (ts.wrapping_add(phase_ms as u32) % period_ms as u32) as f32;
    if effect == Effect::Blinking {
        if t < period_ms as f32 / 3.0 { 1.0 } else { 0.0 }
    } else {
        // the fork uses a fixed 5 s breathing period; sin(x) = cos(x - pi/2)
        let x = t / 5000.0 * 2.0 * core::f32::consts::PI;
        0.5 * (1.0 + cosf(x - core::f32::consts::FRAC_PI_2))
    }
}

fn scale(c: Rgb, k: f32) -> Rgb {
    Rgb::new((c.r as f32 * k) as u8, (c.g as f32 * k) as u8, (c.b as f32 * k) as u8)
}

#[derive(Clone, Copy, Debug, Default)]
struct Signal {
    started_ms: u32,
    duration_ms: u32,
    color: Rgb,
    effect: Effect,
    period_ms: u16,
    phase_ms: u16,
    priority: Priority,
    enhance: bool,
    sync_gps: bool,
}

/// Where the show colour comes from (LightInputSource).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Source {
    #[default]
    Show,
    Fixed(Rgb),
    PixelGrid,
}

/// Interactive "swarm as a display" state: this drone's pixel and palette.
#[derive(Clone, Debug, Default)]
pub struct PixelGrid {
    row: u16,
    col: u16,
    palette: Vec<u8>,
    pub color: Rgb,
}

impl PixelGrid {
    fn palette_color(&self, i: usize) -> Rgb {
        match self.palette.get(i * 3..i * 3 + 3) {
            Some(p) => Rgb::new(p[0], p[1], p[2]),
            None => Rgb::BLACK,
        }
    }

    fn palette_len(&self) -> usize {
        self.palette.len() / 3
    }

    pub fn set_row(&mut self, v: u32) -> bool {
        let ok = v < 4096;
        if ok {
            self.row = v as u16;
        }
        ok
    }

    pub fn set_column(&mut self, v: u32) -> bool {
        let ok = v < 4096;
        if ok {
            self.col = v as u16;
        }
        ok
    }

    /// Applies the show file's GCS light control block (TLV); `None` if the
    /// block is absent. Leaves the state unchanged on a corrupt block.
    pub fn load_block(&mut self, body: Option<&[u8]>) -> bool {
        let mut coords = (0u16, 0u16);
        let mut palette: &[u8] = &[];
        let mut b = body.unwrap_or(&[]);
        while !b.is_empty() {
            if b.len() < 3 {
                return false;
            }
            let (tag, len) = (b[0], u16::from_le_bytes([b[1], b[2]]) as usize);
            let Some(v) = b.get(3..3 + len) else {
                return false;
            };
            match tag {
                0 | 2 => {
                    if len != 4 {
                        return false;
                    }
                    let x = i16::from_le_bytes([v[0], v[1]]);
                    let y = i16::from_le_bytes([v[2], v[3]]);
                    if !(0..4096).contains(&x) || !(0..4096).contains(&y) {
                        return false;
                    }
                    if tag == 0 {
                        coords = (x as u16, y as u16);
                    }
                }
                1 => {
                    if len % 3 != 0 {
                        return false;
                    }
                    palette = v;
                }
                _ => {}
            }
            b = &b[3 + len..];
        }
        self.palette = palette.to_vec();
        self.col = coords.0;
        self.row = coords.1;
        true
    }

    /// Decodes a pixel grid update (TUNNEL payload type 42421) into this
    /// drone's pixel colour. Returns whether the message was well formed.
    pub fn handle_update(&mut self, p: &[u8]) -> bool {
        const HDR: usize = 5;
        if p.len() < HDR {
            return false;
        }
        let version = p[0] >> 6;
        let encoding = (p[0] >> 4) & 3;
        let width = ((p[0] & 0x0f) as u32) << 8 | p[1] as u32;
        let x = (p[2] as u32) << 4 | (p[3] >> 4) as u32;
        let y = ((p[3] & 0x0f) as u32) << 8 | p[4] as u32;
        if version != 0 || width == 0 {
            return false;
        }
        let bpp = [1u32, 4, 8, 16][encoding as usize];
        let num_bits = (p.len() - HDR) as u32 * 8;
        let height = num_bits / (width * bpp);
        if height == 0 || num_bits - height * width * bpp >= 8 {
            return false;
        }
        let (row, col) = (self.row as u32, self.col as u32);
        if row < y || col < x || row - y >= height || col - x >= width {
            return true;
        }
        let i = ((row - y) * width + (col - x)) as usize;
        let d = &p[HDR..];
        self.color = match encoding {
            0 => {
                let bit = (d[i / 8] >> (7 - i % 8)) & 1 != 0;
                match self.palette_len() {
                    0 if bit => Rgb::WHITE,
                    0 | 1 if !bit => Rgb::BLACK,
                    1 => self.palette_color(0),
                    _ => self.palette_color(bit as usize),
                }
            }
            1 => self.palette_color(if i % 2 == 0 { d[i / 2] >> 4 } else { d[i / 2] & 0x0f } as usize),
            2 => self.palette_color(d[i] as usize),
            _ => Rgb::decode_rgb565(u16::from_be_bytes([d[i * 2], d[i * 2 + 1]])),
        };
        true
    }
}

/// Per-tick inputs, filled by the app at 25 Hz.
#[derive(Clone, Copy, Debug, Default)]
pub struct LightInputs {
    /// app::Stage as u8
    pub stage: u8,
    /// current flight mode number
    pub mode: u8,
    /// NTF_* bits
    pub notify: u32,
    /// motor spool state is not SHUT_DOWN
    pub spooling: bool,
    /// bubble fence breached with action FLASH_LIGHTS
    pub bubble_flash: bool,
    /// SHOW_START_AUTH
    pub auth: u8,
    pub preflight_failures: u8,
    pub performance_completed: bool,
    pub start_scheduled: bool,
    pub time_to_takeoff_sec: f32,
    /// negative before the show start
    pub elapsed_sec: f32,
    /// light program colour at `elapsed_sec`, if set
    pub show_colour: Option<Rgb>,
    /// GPS-synced ms (boot ms if GPS time is not valid)
    pub gps_ms: u32,
    /// boot ms
    pub now_ms: u32,
    /// SHOW_PRE_LIGHTS
    pub pre_lights: i32,
    /// SHOW_OPTIONS
    pub options: i32,
}

/// Colour decided for this tick, before gamma and RGBW conversion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Frame {
    pub rgb: Rgb,
    /// grey "where are you" signal: drive the W channel at the same level
    pub enhance: bool,
}

#[derive(Clone, Debug, Default)]
pub struct Lights {
    signal: Signal,
    pub source: Source,
    pub pixel: PixelGrid,
    /// last colour for telemetry and logs
    pub last_color: Rgb,
    last_stage: u8,
    rng: u32,
}

impl Lights {
    fn random16(&mut self, seed: u32) -> u16 {
        // xorshift32; seeded lazily from the clock
        if self.rng == 0 {
            self.rng = seed | 1;
        }
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng = x;
        (x >> 16) as u16
    }

    fn flash(&mut self, color: Rgb, count: u32, priority: Priority, enhance: bool, now_ms: u32) {
        // 100 ms on, 200 ms off
        self.signal = Signal {
            started_ms: now_ms.max(1),
            duration_ms: count * 300 - 200,
            color,
            effect: Effect::Blinking,
            period_ms: 300,
            phase_ms: 0,
            priority,
            enhance,
            sync_gps: true,
        };
    }

    /// Compass calibration saved.
    pub fn flash_success(&mut self, now_ms: u32) {
        self.flash(GREEN, 3, Priority::Internal, false, now_ms);
    }

    /// Compass calibration failed.
    pub fn flash_failure(&mut self, now_ms: u32) {
        self.flash(RED, 3, Priority::Internal, false, now_ms);
    }

    /// LED_CONTROL with instance = pattern = 42. `payload` is the full
    /// message payload; `target_system` its target (0 = broadcast); `group`
    /// is SHOW_GROUP. Returns false if the message is not ours.
    pub fn handle_led_control(&mut self, payload: &[u8], target_system: u8, group: u8, now_ms: u32) -> bool {
        let b = |i: usize| payload.get(i).copied().unwrap_or(0);
        let cb = |i: usize| b(5 + i);
        if b(2) != LED_CONTROL_MAGIC || b(3) != LED_CONTROL_MAGIC {
            return false;
        }
        let len = b(4);
        if len == 4 || len > 8 {
            return false;
        }
        let priority = if target_system == 0 { Priority::Broadcast } else { Priority::Individual };
        if priority < self.signal.priority {
            if now_ms.wrapping_sub(self.signal.started_ms) > self.signal.duration_ms {
                self.signal.priority = Priority::None;
            } else {
                // a higher priority signal is still playing
                return true;
            }
        }
        let mask = |m: u8| m == 0 || 1u32.checked_shl(group as u32).is_some_and(|bit| m as u32 & bit != 0);
        let index = |lo: u8, hi: u8| u16::from_le_bytes([lo, hi]) == group as u16;
        let matches = match len {
            1 => mask(cb(0)),
            2 => index(cb(0), cb(1)),
            7 => mask(cb(6)),
            8 => index(cb(6), cb(7)),
            _ => true,
        };
        // the fork restarts the previous signal on a group mismatch; ignore instead
        if !matches {
            return true;
        }
        let color = Rgb::new(cb(0), cb(1), cb(2));
        let duration = u16::from_le_bytes([cb(3), cb(4)]) as u32;
        let random_phase = self.random16(now_ms) % 5000;
        let s = &mut self.signal;
        s.started_ms = now_ms.max(1);
        s.priority = priority;
        s.enhance = false;
        // broadcasts are GPS-synced so the swarm flashes together
        s.sync_gps = target_system == 0;
        match len {
            0..=2 => self.flash(Rgb::WHITE, 5, priority, true, now_ms),
            3 | 5 => {
                s.duration_ms = if len == 3 { 5000 } else { duration };
                s.color = color;
                s.effect = Effect::Solid;
                s.period_ms = 0;
                s.phase_ms = 0;
            }
            _ => {
                s.duration_ms = duration;
                s.period_ms = 5000;
                s.color = color;
                // keep the phase if the effect is unchanged
                let effect = Effect::from_u8(cb(5));
                if s.effect != effect || cb(5) > 3 {
                    s.effect = effect;
                    s.phase_ms = if effect == Effect::Breathing { random_phase } else { 0 };
                }
            }
        }
        if self.signal.duration_ms == 0 {
            self.signal.started_ms = 0;
            self.signal.effect = Effect::Off;
        }
        true
    }

    /// MAV_CMD_USER_1 param1=4: set the light input source. Returns a MAV_RESULT.
    pub fn handle_source_command(&mut self, param2: f32, param3: f32, x: i32) -> u8 {
        if param2 != 0.0 {
            return 3; // UNSUPPORTED: only light 0
        }
        if param3 != (param3 as i32) as f32 {
            return 4;
        }
        self.source = match param3 as i32 {
            0 => Source::Fixed(Rgb::new((x >> 16) as u8, (x >> 8) as u8, x as u8)),
            1 => Source::Show,
            2 => Source::PixelGrid,
            _ => return 4, // FAILED
        };
        0
    }

    /// Show-file LIGHT event SET_SOURCE payload `[light=0, src, rgb565_hi, rgb565_lo]`.
    pub fn configure_source_from_event(&mut self, p: &[u8]) -> bool {
        if p.len() < 4 || p[0] != 0 {
            return false;
        }
        self.source = match p[1] {
            0 => Source::Fixed(Rgb::decode_rgb565(u16::from_be_bytes([p[2], p[3]]))),
            1 => Source::Show,
            2 => Source::PixelGrid,
            _ => return false,
        };
        true
    }

    fn desired_color(&self, i: &LightInputs) -> Rgb {
        match self.source {
            Source::Fixed(c) => c,
            Source::PixelGrid => self.pixel.color,
            Source::Show if i.elapsed_sec >= 0.0 => i.show_colour.unwrap_or(Rgb::BLACK),
            Source::Show => WHITE_DIM,
        }
    }

    fn track_stage(&mut self, i: &LightInputs) {
        if i.stage == self.last_stage {
            return;
        }
        self.last_stage = i.stage;
        match i.stage {
            STAGE_WAIT_FOR_START => {
                self.source = if i.options & OPT_PIXEL_GRID_ON_GROUND != 0 { Source::PixelGrid } else { Source::Show };
            }
            STAGE_PERFORMING | STAGE_TESTING_LIGHTS | STAGE_LANDED => self.source = Source::Show,
            _ => {}
        }
    }

    /// The fork's _update_lights() colour selection, 25 Hz.
    pub fn update(&mut self, i: &LightInputs) -> Frame {
        self.track_stage(i);
        let flag = |b: u32| i.notify & b != 0;
        let flying = flag(NTF_FLYING);
        let show = i.mode == MODE_SHOW;
        let landing = i.mode == MODE_LAND || (show && i.stage == STAGE_LANDING);
        let landed = !flying && (i.mode == MODE_LAND || (show && i.stage == STAGE_LANDED));
        let rtl = i.mode == MODE_RTL || i.mode == MODE_SMART_RTL || (show && i.stage == STAGE_RTL);

        let mut color = Rgb::BLACK;
        let mut dimmable = true;
        let mut enhance = false;
        let mut failsafe = false;
        let mut brightness = i.pre_lights;
        let mut pattern = ALL_ON;
        let mut pulse = 0.0f32;

        if flag(NTF_COMPASS_CAL_RUNNING) {
            color = MAGENTA;
            pulse = 0.5;
            brightness = brightness.max(1);
        } else if self.signal.started_ms != 0 {
            let s = self.signal;
            let diff = i.now_ms.wrapping_sub(s.started_ms);
            if (diff as i32) < 0 || diff > s.duration_ms {
                self.signal.started_ms = 0;
                self.signal.priority = Priority::None;
            } else {
                let ts = if s.sync_gps { i.gps_ms } else { diff };
                color = scale(s.color, modulation(ts, s.effect, s.period_ms, s.phase_ms));
                enhance = s.enhance;
            }
            dimmable = false;
        } else if flag(NTF_EKF_BAD | NTF_FAILSAFE_BATTERY | NTF_FAILSAFE_GCS | NTF_FAILSAFE_RADIO) {
            color = RED;
            pattern = BLINK;
            failsafe = true;
        } else if i.bubble_flash {
            dimmable = false;
            color = RED;
            pattern = FLASH_FOUR_TIMES_PER_SECOND;
            failsafe = true;
        } else if flying {
            dimmable = false;
            if rtl {
                color = ORANGE;
                pattern = BLINK;
                failsafe = true;
            } else if landing {
                color = ORANGE;
                failsafe = true;
            } else if show {
                match i.stage {
                    STAGE_ERROR => {
                        color = RED;
                        pattern = BLINK;
                        failsafe = true;
                    }
                    STAGE_LOITER => color = Rgb::WHITE,
                    _ => color = self.desired_color(i),
                }
            } else {
                color = Rgb::WHITE;
            }
        } else if i.spooling {
            if landing {
                color = ORANGE;
                pattern = FLASH_FOUR_TIMES_PER_SECOND;
            } else if show && i.elapsed_sec >= 0.0 {
                color = self.desired_color(i);
                dimmable = false;
            } else {
                color = GREEN;
                pattern = BLINK_TWICE_PER_SECOND;
            }
        } else if flag(NTF_INITIALISING) || !flag(NTF_PRE_ARM_CHECK) || !flag(NTF_PRE_ARM_GPS_CHECK) {
            color = YELLOW;
            pattern = FLASH_ONCE_PER_SECOND;
        } else if landed {
            color = GREEN_DIM;
        } else if show {
            if i.stage == STAGE_ERROR {
                color = RED;
                pattern = BLINK;
            } else if i.preflight_failures != 0 {
                color = YELLOW;
                pulse = 0.5;
            } else if i.auth != AUTH_REVOKED {
                let mut idle = false;
                if i.performance_completed {
                    color = GREEN_DIM;
                    idle = true;
                } else if i.time_to_takeoff_sec > 10.0 {
                    color = GREEN_DIM;
                    pulse = 0.5;
                    idle = true;
                } else if i.auth == AUTH_LIVE {
                    color = GREEN;
                    pattern = FLASH_TWICE_PER_SECOND;
                } else {
                    color = self.desired_color(i);
                    dimmable = false;
                }
                // let the pixel grid show through while idling on the ground
                if idle && self.source == Source::PixelGrid {
                    color = self.desired_color(i);
                    dimmable = false;
                }
            } else {
                color = LIGHT_BLUE;
                pulse = if i.start_scheduled && i.time_to_takeoff_sec >= 0.0 { 0.5 } else { 0.3 };
            }
        } else {
            color = GREEN_DIM;
        }

        if pulse > 0.0 {
            color = scale(color, pulse * modulation(i.gps_ms, Effect::Breathing, 5000, 0));
        } else if pattern != ALL_ON && pattern & (0x80 >> ((i.gps_ms % 1000) / 125)) == 0 {
            color = Rgb::BLACK;
        }

        if dimmable {
            if brightness < 4 {
                let shift = match brightness {
                    i32::MIN..=0 => 8,
                    1 => 2,
                    2 => 1,
                    _ => 0,
                };
                let sh = |v: u8| ((v as u32) >> shift) as u8;
                color = Rgb::new(sh(color.r), sh(color.g), sh(color.b));
            } else if brightness <= 100 {
                color = scale(color, brightness as f32 / 100.0);
            }
        }

        if failsafe && flying && i.options & OPT_DISABLE_FAILSAFE_LIGHTS != 0 {
            color = Rgb::BLACK;
        }

        self.last_color = color;
        Frame { rgb: color, enhance }
    }
}

/// DroneShowLED output stage: change detection, gamma LUT, minimum
/// brightness cutoff and repeat-until-sent.
#[derive(Clone, Debug)]
pub struct LedOutput {
    gamma: f32,
    lut: [u8; 256],
    min_scaled: u8,
    last: Rgbw,
    repeat_left: u8,
    conv: Option<(f32, RgbwConversion)>,
}

impl Default for LedOutput {
    fn default() -> Self {
        let mut o = LedOutput {
            gamma: 0.0,
            lut: [0; 256],
            min_scaled: 0,
            last: Rgbw::default(),
            repeat_left: 1,
            conv: None,
        };
        o.set_gamma(1.0);
        o
    }
}

impl LedOutput {
    pub fn set_gamma(&mut self, g: f32) {
        if (g - self.gamma).abs() <= f32::EPSILON {
            return;
        }
        self.gamma = g;
        for (i, v) in self.lut.iter_mut().enumerate() {
            *v = if (g - 1.0).abs() > f32::EPSILON && g > 0.0 && g < 10.0 {
                if i == 0 { 0 } else { roundf(powf(i as f32 / 255.0, g) * 255.0).clamp(0.0, 255.0) as u8 }
            } else if g >= 10.0 {
                if i == 255 { 255 } else { 0 }
            } else {
                i as u8
            };
        }
        self.repeat_left = 1;
    }

    pub fn set_min_brightness(&mut self, v: f32) {
        self.min_scaled = if !(v >= 0.0) {
            0
        } else if v < 1.0 {
            (v * 255.0) as u8
        } else {
            255
        };
    }

    /// Pre-gamma colour; schedules a send if it changed.
    pub fn set_rgbw(&mut self, c: Rgbw) {
        if c != self.last {
            self.last = c;
            self.repeat_left = 1;
        }
    }

    /// RGB or RGBW conversion as the fork, then `set_rgbw`.
    pub fn set_frame(&mut self, f: Frame, has_white: bool, wtemp: f32) {
        let c = f.rgb;
        if !has_white {
            self.set_rgbw(Rgbw { r: c.r, g: c.g, b: c.b, w: 0 });
            return;
        }
        let conv = if f.enhance && c.r == c.g && c.g == c.b {
            RgbwConversion::FixedValue(c.r)
        } else if wtemp > 0.0 {
            match self.conv {
                Some((t, conv)) if t == wtemp => conv,
                _ => {
                    let conv = RgbwConversion::color_temperature(wtemp);
                    self.conv = Some((wtemp, conv));
                    conv
                }
            }
        } else {
            RgbwConversion::SubtractMin
        };
        self.set_rgbw(c.to_rgbw(&conv));
    }

    /// Gamma-corrected value to send, if a send is pending.
    pub fn pending(&self) -> Option<Rgbw> {
        if self.repeat_left == 0 {
            return None;
        }
        let l = self.last;
        if l.r.max(l.g).max(l.b).max(l.w) < self.min_scaled {
            return Some(Rgbw::default());
        }
        let g = |v: u8| self.lut[v as usize];
        Some(Rgbw { r: g(l.r), g: g(l.g), b: g(l.b), w: g(l.w) })
    }

    /// The pending value was delivered.
    pub fn sent(&mut self) {
        self.repeat_left = self.repeat_left.saturating_sub(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ground_ok() -> LightInputs {
        LightInputs {
            notify: NTF_PRE_ARM_CHECK | NTF_PRE_ARM_GPS_CHECK,
            pre_lights: 3,
            now_ms: 1000,
            ..Default::default()
        }
    }

    #[test]
    fn modulation_effects() {
        assert_eq!(modulation(0, Effect::Blinking, 300, 0), 1.0);
        assert_eq!(modulation(100, Effect::Blinking, 300, 0), 0.0);
        assert!((modulation(1250, Effect::Breathing, 5000, 0) - 1.0).abs() < 1e-5);
        assert!(modulation(3750, Effect::Breathing, 5000, 0).abs() < 1e-5);
        assert_eq!(modulation(7, Effect::Off, 5000, 0), 0.0);
    }

    #[test]
    fn standby_and_dimming() {
        let mut l = Lights::default();
        let mut i = ground_ok();
        assert_eq!(l.update(&i).rgb, GREEN_DIM);
        i.pre_lights = 1;
        assert_eq!(l.update(&i).rgb, Rgb::new(0, 32, 0));
        i.pre_lights = 50;
        assert_eq!(l.update(&i).rgb, Rgb::new(0, 64, 0));
        i.pre_lights = 0;
        assert_eq!(l.update(&i).rgb, Rgb::BLACK);
    }

    #[test]
    fn prearm_flash_and_failsafe() {
        let mut l = Lights::default();
        let mut i = ground_ok();
        i.notify = 0;
        i.gps_ms = 50;
        assert_eq!(l.update(&i).rgb, YELLOW);
        i.gps_ms = 200;
        assert_eq!(l.update(&i).rgb, Rgb::BLACK);
        i.notify = NTF_FLYING | NTF_FAILSAFE_BATTERY;
        i.gps_ms = 10;
        assert_eq!(l.update(&i).rgb, RED);
        i.options = OPT_DISABLE_FAILSAFE_LIGHTS;
        assert_eq!(l.update(&i).rgb, Rgb::BLACK);
    }

    #[test]
    fn flying_show_colour() {
        let mut l = Lights::default();
        let mut i = ground_ok();
        i.notify |= NTF_FLYING;
        i.mode = MODE_SHOW;
        i.stage = STAGE_PERFORMING;
        i.elapsed_sec = 3.0;
        i.show_colour = Some(Rgb::new(1, 2, 3));
        i.pre_lights = 0;
        assert_eq!(l.update(&i).rgb, Rgb::new(1, 2, 3));
        i.elapsed_sec = -1.0;
        assert_eq!(l.update(&i).rgb, WHITE_DIM);
    }

    fn led_control(target: u8, custom: &[u8]) -> Vec<u8> {
        let mut p = alloc::vec![0u8; 29];
        p[0] = target;
        p[2] = 42;
        p[3] = 42;
        p[4] = custom.len() as u8;
        p[5..5 + custom.len()].copy_from_slice(custom);
        p
    }

    #[test]
    fn led_control_priorities() {
        let mut l = Lights::default();
        let mut i = ground_ok();
        assert!(!l.handle_led_control(&[0, 0, 1, 42, 0], 0, 0, 1000));
        assert!(l.handle_led_control(&led_control(5, &[10, 20, 30]), 5, 0, 1000));
        i.now_ms = 2000;
        assert_eq!(l.update(&i).rgb, Rgb::new(10, 20, 30));
        // broadcast blocked by the individual signal
        assert!(l.handle_led_control(&led_control(0, &[1, 1, 1, 0x10, 0x27]), 0, 0, 2000));
        assert_eq!(l.update(&i).rgb, Rgb::new(10, 20, 30));
        // group mask mismatch leaves the signal alone
        assert!(l.handle_led_control(&led_control(5, &[9, 9, 9, 0x10, 0x27, 1, 0b10]), 5, 0, 2000));
        assert_eq!(l.update(&i).rgb, Rgb::new(10, 20, 30));
        // zero duration cancels
        assert!(l.handle_led_control(&led_control(5, &[0, 0, 0, 0, 0]), 5, 0, 2000));
        assert_eq!(l.update(&i).rgb, GREEN_DIM);
        // individual signal expiry
        assert!(l.handle_led_control(&led_control(5, &[10, 20, 30]), 5, 0, 3000));
        i.now_ms = 8001;
        // as the fork, the tick that ends a signal is black
        assert_eq!(l.update(&i).rgb, Rgb::BLACK);
        assert_eq!(l.update(&i).rgb, GREEN_DIM);
    }

    #[test]
    fn where_are_you_enhances() {
        let mut l = Lights::default();
        let mut i = ground_ok();
        assert!(l.handle_led_control(&led_control(0, &[]), 0, 0, 1000));
        i.gps_ms = 3000;
        let f = l.update(&i);
        assert_eq!(f, Frame { rgb: Rgb::WHITE, enhance: true });
        let mut o = LedOutput::default();
        o.set_frame(f, true, 0.0);
        assert_eq!(o.pending(), Some(Rgbw { r: 255, g: 255, b: 255, w: 255 }));
    }

    #[test]
    fn output_gamma_minbri_repeat() {
        let mut o = LedOutput::default();
        o.set_gamma(2.0);
        o.set_rgbw(Rgbw { r: 128, g: 255, b: 0, w: 0 });
        assert_eq!(o.pending(), Some(Rgbw { r: 64, g: 255, b: 0, w: 0 }));
        o.sent();
        assert_eq!(o.pending(), None);
        o.set_min_brightness(0.5);
        o.set_rgbw(Rgbw { r: 100, g: 0, b: 0, w: 0 });
        assert_eq!(o.pending(), Some(Rgbw::default()));
    }

    #[test]
    fn pixel_grid_decode() {
        let mut g = PixelGrid::default();
        // coords x=1 y=0, palette red/blue
        let block = [0, 4, 0, 1, 0, 0, 0, 1, 6, 0, 255, 0, 0, 0, 0, 255];
        assert!(g.load_block(Some(&block)));
        // 1 bpp, width 2, origin 0,0: pixels 0b01 -> x=1 is set
        assert!(g.handle_update(&[0x00, 0x02, 0, 0, 0, 0b0100_0000]));
        assert_eq!(g.color, Rgb::new(0, 0, 255));
        // RGB565, width 2
        assert!(g.handle_update(&[0x30, 0x02, 0, 0, 0, 0, 0, 0xf8, 0x00]));
        assert_eq!(g.color, Rgb::new(0xf8, 0, 0));
        assert!(!g.handle_update(&[0x30, 0x02, 0, 0, 0, 0, 0]));
    }
}

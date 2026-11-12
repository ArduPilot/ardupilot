// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush lights/* (executor, loop stack, transition, player,
// program, colors) (Copyright CollMot Robotics Ltd).
//! Light program bytecode (block 2), its player and colour helpers.
//! Durations are LEB128 varints in 20 ms units; times are ms.

use alloc::vec::Vec;

use super::binary::{ShowFile, BLOCK_LIGHT_PROGRAM};
use super::fmath::{logf, powf};
use super::Result;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Rgbw {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub w: u8,
}

impl Rgb {
    pub const BLACK: Rgb = Rgb::new(0, 0, 0);
    pub const WHITE: Rgb = Rgb::new(255, 255, 255);

    pub const fn new(r: u8, g: u8, b: u8) -> Self {
        Rgb { r, g, b }
    }

    pub fn decode_rgb565(c: u16) -> Self {
        Rgb::new(((c & 0xf800) >> 8) as u8, ((c & 0x7e0) >> 3) as u8, ((c & 0x1f) << 3) as u8)
    }

    pub fn encode_rgb565(self) -> u16 {
        ((self.r as u16 >> 3) & 0x1f) << 11 | ((self.g as u16 >> 2) & 0x3f) << 5 | ((self.b as u16 >> 3) & 0x1f)
    }

    pub fn lerp(self, other: Rgb, ratio: f32) -> Rgb {
        let f = |a: u8, b: u8| component(a as f32 + (b as i32 - a as i32) as f32 * ratio);
        Rgb::new(f(self.r, other.r), f(self.g, other.g), f(self.b, other.b))
    }

    /// Approximate RGB of a black body at `temperature` kelvin.
    pub fn from_color_temperature(temperature: f32) -> Rgb {
        let t = if temperature < 1000.0 {
            10.0
        } else if temperature > 40000.0 {
            400.0
        } else {
            temperature / 100.0
        };
        let r = if t <= 66.0 { 255 } else { component(329.698727446 * powf(t - 60.0, -0.1332047592)) };
        let g = if t <= 66.0 {
            component(99.4708025861 * logf(t) - 161.1195681661)
        } else {
            component(288.1221695283 * powf(t - 60.0, -0.0755148492))
        };
        let b = if t >= 66.0 { 255 } else { component(138.5177312231 * logf(t - 10.0) - 305.0447927307) };
        Rgb::new(r, g, b)
    }

    pub fn to_rgbw(self, conv: &RgbwConversion) -> Rgbw {
        let (r, g, b) = (self.r, self.g, self.b);
        match *conv {
            RgbwConversion::SubtractMin => {
                let v = r.min(g).min(b);
                Rgbw { r: r - v, g: g - v, b: b - v, w: v }
            }
            RgbwConversion::FixedValue(w) => Rgbw { r, g, b, w },
            RgbwConversion::Reference { mul, div, .. } => {
                let vals = [r, g, b];
                let mut min_scaled = f32::INFINITY;
                for i in 0..3 {
                    if mul[i] > 0.0 {
                        let s = vals[i] as f32 * mul[i];
                        if s < min_scaled {
                            min_scaled = s;
                        }
                    }
                }
                // infinite: the white LED emits none of the channels
                let w = if min_scaled.is_infinite() || min_scaled <= 0.0 {
                    0
                } else if min_scaled <= 255.0 {
                    min_scaled as u8
                } else {
                    255
                };
                let mut out = [0u8; 3];
                for i in 0..3 {
                    let corr = w as f32 * div[i];
                    out[i] = if vals[i] as f32 > corr { (vals[i] as f32 - corr) as u8 } else { 0 };
                }
                Rgbw { r: out[0], g: out[1], b: out[2], w }
            }
        }
    }
}

/// Truncates towards zero, saturating; NaN becomes 0.
fn component(v: f32) -> u8 {
    if !(v > 0.0) {
        0
    } else if v > 255.0 {
        255
    } else {
        v as u8
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RgbwConversion {
    FixedValue(u8),
    SubtractMin,
    /// White LED of the given reference colour; temperature 0 if not from a temperature.
    Reference { mul: [f32; 3], div: [f32; 3], temperature: f32 },
}

impl Default for RgbwConversion {
    fn default() -> Self {
        RgbwConversion::FixedValue(0)
    }
}

impl RgbwConversion {
    pub fn reference_color(reference: Rgb) -> Self {
        let ch = [reference.r, reference.g, reference.b];
        let max = ch.iter().fold(1.0f32, |m, &c| if c as f32 > m { c as f32 } else { m });
        let mut mul = [0.0f32; 3];
        let mut div = [0.0f32; 3];
        for i in 0..3 {
            mul[i] = if ch[i] >= 1 { max / ch[i] as f32 } else { 0.0 };
            div[i] = if mul[i] > 0.0 { 1.0 / mul[i] } else { 0.0 };
        }
        RgbwConversion::Reference { mul, div, temperature: 0.0 }
    }

    pub fn color_temperature(temperature: f32) -> Self {
        match Self::reference_color(Rgb::from_color_temperature(temperature)) {
            RgbwConversion::Reference { mul, div, .. } => RgbwConversion::Reference { mul, div, temperature },
            other => other,
        }
    }
}

const CMD_END: u8 = 0x00;
const CMD_NOP: u8 = 0x01;
const CMD_SLEEP: u8 = 0x02;
const CMD_WAIT_UNTIL: u8 = 0x03;
const CMD_SET_COLOR: u8 = 0x04;
const CMD_SET_GRAY: u8 = 0x05;
const CMD_SET_BLACK: u8 = 0x06;
const CMD_SET_WHITE: u8 = 0x07;
const CMD_FADE_TO_COLOR: u8 = 0x08;
const CMD_FADE_TO_GRAY: u8 = 0x09;
const CMD_FADE_TO_BLACK: u8 = 0x0A;
const CMD_FADE_TO_WHITE: u8 = 0x0B;
const CMD_LOOP_BEGIN: u8 = 0x0C;
const CMD_LOOP_END: u8 = 0x0D;

const MAX_LOOP_DEPTH: usize = 4;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct LightProgram {
    bytecode: Vec<u8>,
}

impl LightProgram {
    pub fn from_bytes(bytecode: &[u8]) -> Self {
        LightProgram { bytecode: bytecode.to_vec() }
    }

    pub fn from_show(file: &ShowFile) -> Result<Self> {
        Ok(Self::from_bytes(file.find_first(BLOCK_LIGHT_PROGRAM)?))
    }

    /// Program that shows one colour forever.
    pub fn constant_color(c: Rgb) -> Self {
        LightProgram { bytecode: alloc::vec![CMD_SET_COLOR, c.r, c.g, c.b, CMD_END] }
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytecode
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct LoopItem {
    start: usize,
    iterations_left_plus_one: u8,
    iteration_start: u64,
}

// The C executor uses `unsigned long` clocks; u64 here (as on 64-bit SITL).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct Executor {
    pc: usize,
    ended: bool,
    loops: [LoopItem; MAX_LOOP_DEPTH],
    depth: usize,
    cumulative: u64,
    command_start: u64,
    clock_reset_time: u64,
    next_wakeup: u64,
    reset_clock: bool,
    trans_active: bool,
    trans_start: u64,
    trans_duration: u64,
    start_color: Rgb,
    end_color: Rgb,
    color: Rgb,
}

impl Executor {
    fn rewind(&mut self, code: &[u8]) {
        self.pc = 0;
        self.ended = code.is_empty();
        self.depth = 0;
        self.trans_active = false;
        self.color = Rgb::BLACK;
        self.reset_clock = true;
    }

    fn next_byte(&mut self, code: &[u8]) -> u8 {
        match code.get(self.pc) {
            Some(&b) => {
                self.pc += 1;
                b
            }
            None => CMD_END,
        }
    }

    fn next_varint(&mut self, code: &[u8]) -> u64 {
        let mut result: u32 = 0;
        let mut index = 0;
        let mut overflow = false;
        loop {
            let byte = self.next_byte(code);
            let value = (byte & 0x7f) as u32;
            if index < 4 || (index == 4 && value < 0x10) {
                result |= value << (7 * index);
                index += 1;
            } else {
                overflow = true;
            }
            if byte & 0x80 == 0 {
                break;
            }
        }
        if overflow { u32::MAX as u64 } else { result as u64 }
    }

    fn delay_until(&mut self, ms: u64) {
        let abs = self.clock_reset_time.wrapping_add(ms);
        if self.next_wakeup < abs {
            self.next_wakeup = abs;
        }
    }

    fn handle_delay(&mut self, code: &[u8]) {
        let d = self.next_varint(code) * 20;
        self.cumulative = self.cumulative.wrapping_add(d);
        self.delay_until(self.cumulative);
    }

    fn set_color_reset_transition(&mut self, c: Rgb) {
        self.color = c;
        self.start_color = c;
    }

    // Steps the transition; returns whether it is still active.
    fn step_transition(&mut self, clock: u64) -> bool {
        let progress = if clock < self.trans_start {
            0.0
        } else if self.trans_duration == 0 {
            1.0
        } else {
            let r = (clock - self.trans_start) as f32 / self.trans_duration as f32;
            if r > 1.0 { 1.0 } else { r }
        };
        self.color = self.start_color.lerp(self.end_color, progress);
        self.trans_active = progress < 1.0;
        self.trans_active
    }

    fn fade_to(&mut self, code: &[u8], c: Rgb) {
        let now = self.command_start;
        self.handle_delay(code);
        self.end_color = c;
        self.trans_start = self.command_start;
        self.trans_duration = self.next_wakeup.wrapping_sub(now);
        self.trans_active = true;
        if !self.step_transition(now) {
            self.start_color = self.end_color;
        }
    }

    fn read_rgb(&mut self, code: &[u8]) -> Rgb {
        let r = self.next_byte(code);
        let g = self.next_byte(code);
        let b = self.next_byte(code);
        Rgb::new(r, g, b)
    }

    fn execute_next(&mut self, code: &[u8]) {
        match self.next_byte(code) {
            CMD_END => self.ended = true,
            CMD_NOP => {}
            CMD_SLEEP => self.handle_delay(code),
            CMD_WAIT_UNTIL => {
                let deadline = self.next_varint(code) * 20;
                self.delay_until(deadline);
                self.cumulative = self.next_wakeup.wrapping_sub(self.clock_reset_time);
            }
            CMD_SET_COLOR => {
                let c = self.read_rgb(code);
                self.handle_delay(code);
                self.set_color_reset_transition(c);
            }
            CMD_SET_GRAY => {
                let v = self.next_byte(code);
                self.handle_delay(code);
                self.set_color_reset_transition(Rgb::new(v, v, v));
            }
            CMD_SET_BLACK => {
                self.handle_delay(code);
                self.set_color_reset_transition(Rgb::BLACK);
            }
            CMD_SET_WHITE => {
                self.handle_delay(code);
                self.set_color_reset_transition(Rgb::WHITE);
            }
            CMD_FADE_TO_COLOR => {
                let c = self.read_rgb(code);
                self.fade_to(code, c);
            }
            CMD_FADE_TO_GRAY => {
                let v = self.next_byte(code);
                self.fade_to(code, Rgb::new(v, v, v));
            }
            CMD_FADE_TO_BLACK => self.fade_to(code, Rgb::BLACK),
            CMD_FADE_TO_WHITE => self.fade_to(code, Rgb::WHITE),
            CMD_LOOP_BEGIN => {
                let iterations = self.next_byte(code);
                if self.depth == MAX_LOOP_DEPTH {
                    // nested too deep
                    self.ended = true;
                } else {
                    self.loops[self.depth] = LoopItem {
                        start: self.pc,
                        iterations_left_plus_one: iterations,
                        iteration_start: self.cumulative,
                    };
                    self.depth += 1;
                }
            }
            CMD_LOOP_END => self.loop_end(),
            _ => self.ended = true,
        }
    }

    fn loop_end(&mut self) {
        if self.depth == 0 {
            return;
        }
        let top = &mut self.loops[self.depth - 1];
        if top.iteration_start == self.cumulative {
            // zero-duration iteration: skip the rest, refuse infinite loops
            if top.iterations_left_plus_one == 0 {
                self.ended = true;
            } else {
                self.depth -= 1;
            }
            return;
        }
        match top.iterations_left_plus_one {
            0 => {
                top.iteration_start = self.cumulative;
                self.pc = top.start;
            }
            1 => {
                top.iterations_left_plus_one = 0;
                self.depth -= 1;
            }
            _ => {
                top.iterations_left_plus_one -= 1;
                top.iteration_start = self.cumulative;
                self.pc = top.start;
            }
        }
    }

    fn step(&mut self, code: &[u8], now: u64) -> u64 {
        if self.reset_clock {
            self.clock_reset_time = now;
            self.cumulative = 0;
            self.set_color_reset_transition(Rgb::BLACK);
            self.reset_clock = false;
            self.next_wakeup = now;
        }
        if self.ended {
            self.next_wakeup = now.wrapping_add(60000);
            return self.next_wakeup;
        }
        if self.trans_active && !self.step_transition(now) {
            self.start_color = self.end_color;
        }
        if now >= self.next_wakeup {
            self.command_start = now;
            self.execute_next(code);
        }
        self.next_wakeup
    }
}

/// Plays a light program; the program is passed to each call. Seeking
/// backwards replays from the start.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LightPlayer {
    exec: Executor,
    current: u64,
    next: u64,
}

impl LightPlayer {
    pub fn new(program: &LightProgram) -> Self {
        let mut exec = Executor { ended: true, reset_clock: true, ..Default::default() };
        exec.step(&[], 0);
        exec.rewind(&program.bytecode);
        LightPlayer { exec, current: 0, next: 0 }
    }

    pub fn rewind(&mut self, program: &LightProgram) {
        *self = LightPlayer::new(program);
    }

    /// Advances to `target` ms; returns whether the program has ended.
    pub fn seek(&mut self, program: &LightProgram, target: u64) -> bool {
        let code = &program.bytecode[..];
        if target < self.current {
            self.exec.rewind(code);
            self.current = 0;
            self.next = 0;
        }
        while target > self.next {
            let mut proposal = self.exec.step(code, self.next);
            if proposal < self.next {
                proposal = self.next + 1;
            }
            self.current = self.next;
            self.next = proposal;
        }
        loop {
            self.next = self.exec.step(code, target);
            if self.next > target {
                break;
            }
        }
        self.current = target;
        self.exec.ended
    }

    pub fn color_at(&mut self, program: &LightProgram, t_ms: u64) -> Rgb {
        self.seek(program, t_ms);
        self.exec.color
    }

    /// Next time (ms) at which the output may change.
    pub fn next_timestamp(&self) -> u64 {
        self.next
    }
}

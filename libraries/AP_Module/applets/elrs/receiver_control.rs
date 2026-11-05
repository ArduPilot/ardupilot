// SPDX-License-Identifier: GPL-3.0-or-later
//! Complete receiver policy. Host ABI: opaque 4096-byte/alignment-8 state,
//! 128-byte event, 256-byte actions/publication. No SDK calls on the RF thread.
use crate::protocol::{crc_initializer, parse_binding_uid, uid_seed};
use crate::stream::Stream;
use crate::tracker::Tracker;
fn u32_at(p: &[u8], i: usize) -> u32 {
    u32::from_le_bytes(p[i..i + 4].try_into().unwrap())
}
fn put32(p: &mut [u8], i: usize, n: u32) {
    p[i..i + 4].copy_from_slice(&n.to_le_bytes());
}
pub struct Receiver {
    tracker: Tracker,
    stream: Stream,
    uid: [u8; 6],
    model: u8,
    binding: bool,
    bound: Option<[u8; 6]>,
    error: u8,
    phase: u8,
    started: u32,
    button: bool,
    button_seen: bool,
    attempts: u8,
    recovery_rate: u8,
    recovered: bool,
    recovered_at: u32,
    retry_at: u32,
    configured_rate: u8,
    configured_flrc: bool,
    overruns: u32,
    transmitting: bool,
    tx_at: u32,
    tx_count: u32,
    tx_done: u32,
    tx_timeout: u32,
    generation: u32,
    flushing: bool,
    stream_active: bool,
    led_state: u8,
    led_at: u32,
    version_ok: bool,
    snapshot_cache: [u8; 256],
}
const _: () = assert!(core::mem::size_of::<Receiver>() <= 4096);
const _: () = assert!(core::mem::align_of::<Receiver>() <= 8);
impl Receiver {
    pub fn new(input: &[u8]) -> Self {
        let a = u32_at(input, 20);
        let b = u32_at(input, 24);
        let model = u16::from_le_bytes([input[28], input[29]]);
        Self {
            tracker: Tracker::new(),
            stream: Stream::new(),
            uid: [
                (a >> 16) as u8,
                (a >> 8) as u8,
                a as u8,
                (b >> 16) as u8,
                (b >> 8) as u8,
                b as u8,
            ],
            model: model as u8,
            binding: input[30] == 1,
            bound: None,
            error: if input[31] != 4 || a > 0xffffff || b > 0xffffff || model > 255 {
                1
            } else {
                0
            },
            phase: 0,
            started: u32_at(input, 4),
            button: true,
            button_seen: false,
            attempts: 0,
            recovery_rate: 0,
            recovered: false,
            recovered_at: 0,
            retry_at: 0,
            configured_rate: 255,
            configured_flrc: false,
            overruns: 0,
            transmitting: false,
            tx_at: 0,
            tx_count: 0,
            tx_done: 0,
            tx_timeout: 0,
            generation: 0,
            flushing: true,
            stream_active: false,
            led_state: 0,
            led_at: 0,
            version_ok: input[31] == 4,
            snapshot_cache: [0; 256],
        }
    }
    fn fail(&mut self, now: u32) {
        self.recovery_rate = self.tracker.rate_index;
        self.tracker.stop();
        self.transmitting = false;
        self.error = 8;
        self.phase = 4;
        self.retry_at = now.wrapping_add(1000000);
    }
    fn configured(&mut self, out: &mut [u8]) {
        out[1] |= 2 | 4;
        self.configured_rate = self.tracker.rate_index;
        self.configured_flrc = self.tracker.rate().flrc;
    }
    fn radio(&mut self, input: &[u8], out: &mut [u8], now: u32) {
        if self.error != 0 && self.phase == 0 {
            self.phase = 5;
        }
        if self.phase == 0 {
            if !self.button_seen {
                self.started = now;
                self.button_seen = true;
            }
            self.button &= input[19] != 0;
            if now.wrapping_sub(self.started) < 20000 {
                put32(out, 4, 5000);
                return;
            }
            self.binding |= self.button && self.button_seen;
            if !self.binding && !self.tracker.start(self.uid, self.model, 0) {
                self.error = 2;
                self.phase = 5;
                return;
            }
            self.phase = 1;
        }
        if self.phase == 5 || self.bound.is_some() {
            out[1] = 16;
            put32(out, 4, 5000);
            return;
        }
        if self.phase == 4 {
            if (now.wrapping_sub(self.retry_at) as i32) < 0 {
                put32(out, 4, 5000);
                return;
            }
            if self.attempts == 3 {
                self.error = 9;
                self.phase = 5;
                out[1] = 16;
                return;
            }
            self.attempts += 1;
            self.phase = 1;
        }
        if self.phase == 1 {
            out[1] = 1;
            self.phase = 2;
            return;
        }
        if self.phase == 2 {
            if input[3] == 0 {
                return;
            }
            if input[3] != 1 {
                if self.attempts == 0 {
                    self.error = 3;
                    self.phase = 5;
                } else {
                    self.phase = 4;
                    self.retry_at = now.wrapping_add(1000000);
                }
                return;
            }
            self.overruns = u32_at(input, 12);
            if self.attempts > 0 && !self.binding {
                if !self.tracker.start(self.uid, self.model, self.recovery_rate) {
                    self.error = 2;
                    self.phase = 5;
                    return;
                }
            }
            self.error = 0;
            self.phase = 3;
            self.recovered = self.attempts > 0;
            self.recovered_at = now;
            self.configured(out);
            return;
        }
        if input[2] == 0 || input[3] == 2 {
            self.fail(now);
            out[1] = 16;
            return;
        }
        if self.recovered && now.wrapping_sub(self.recovered_at) >= 60000000 {
            self.attempts = 0;
        }
        if input[3] == 3 && self.transmitting {
            self.transmitting = false;
            self.tx_count = self.tx_count.wrapping_sub(1);
            // The host did not start TX; keep receiving and retain sync, as
            // the original receiver did when a telemetry deadline expired.
        }
        if input[1] == 2 && self.transmitting {
            self.transmitting = false;
            self.tx_done = self.tx_done.wrapping_add(1);
            out[1] |= 4;
        }
        if input[1] == 1 {
            let overruns = u32_at(input, 12);
            if overruns != self.overruns {
                self.overruns = overruns;
                self.tracker.lose();
            } else if self.binding {
                if input[18] == 8 && now.wrapping_sub(u32_at(input, 8)) <= 2000 {
                    self.bound = parse_binding_uid(&input[20..28]);
                }
                if self.bound.is_some() {
                    out[1] = 16;
                    return;
                }
            } else if usize::from(input[18]) == self.tracker.rate().size {
                self.tracker.receive(
                    &input[20..20 + usize::from(input[18])],
                    u32_at(input, 8),
                    now,
                );
            }
        }
        if self.transmitting
            && now.wrapping_sub(self.tx_at) >= u32::from(self.tracker.rate().airtime) + 500
        {
            self.transmitting = false;
            self.tx_timeout = self.tx_timeout.wrapping_add(1);
            self.tracker.lose();
            out[1] |= 16 | 4;
        }
        let slot_due = self.tracker.wait(now) == 0;
        if !self.binding {
            self.tracker.advance(now);
        }
        if self.tracker.rate_index != self.configured_rate {
            if !(self.configured_flrc && self.tracker.rate().flrc) {
                self.configured(out);
            }
            self.configured_rate = self.tracker.rate_index;
        }
        if slot_due && !self.binding && !self.transmitting {
            let mut packet = [0; 13];
            if self
                .tracker
                .telemetry(now, &mut packet, input[16] as i8, input[17] as i8)
            {
                self.transmitting = true;
                self.tx_at = now;
                self.tx_count = self.tx_count.wrapping_add(1);
                out[1] |= 8;
                put32(
                    out,
                    240,
                    now.wrapping_add(u32::from(self.tracker.telemetry_valid_for(now))),
                );
                out[25] = self.tracker.rate().size as u8;
                out[26..39].copy_from_slice(&packet);
            }
        }
        let mut wait = self.tracker.wait(now).min(5000);
        if self.transmitting {
            wait = wait.min(
                (u32::from(self.tracker.rate().airtime) + 500)
                    .saturating_sub(now.wrapping_sub(self.tx_at)),
            );
        }
        put32(out, 4, wait);
    }
    fn main(&mut self, input: &[u8], out: &mut [u8], now: u32) {
        let active = self.phase == 3
            && self.error == 0
            && !self.binding
            && self.tracker.mavlink_mode()
            && self.tracker.denom > 1;
        let reset = self.generation != self.tracker.generation || (self.stream_active && !active);
        if reset {
            self.generation = self.tracker.generation;
            self.flushing = true;
            self.stream.reset();
        }
        self.stream_active = active;
        if !active {
            self.flushing = true;
        }
        let matching = u32_at(input, 84) == self.generation;
        if matching && input[3] != 0 {
            self.flushing = false;
        }
        let ready = active && !self.flushing;
        if input[1] != 0 {
            out[42] = u8::from(
                !matching || !ready || self.tracker.queue(&input[20..20 + usize::from(input[1])]),
            );
        }
        if ready {
            out[3] |= 8;
            if !self.tracker.downlink_pending() {
                out[3] |= 32;
            }
        }
        if self.flushing || !active {
            out[3] |= 16;
        }
        self.stream.update(
            &mut self.tracker,
            now,
            if matching && !reset { input[2] } else { 0 },
            u32_at(input, 12),
            u32_at(input, 8),
            ready,
            out,
        );
    }
    fn snapshot(&mut self, out: &mut [u8], now: u32) {
        out[0] = 1;
        out[2] = self.error;
        if self.error != 0 {
            let text: &[u8] = match self.error {
                1 => b"ELRS: invalid UID, model or protocol version",
                2 => b"ELRS: receiver start failed",
                3 => b"ELRS: SX1280 initialization failed",
                8 => b"ELRS: SX1280 operation failed; retrying",
                9 => b"ELRS: SX1280 recovery exhausted after 3 retries",
                _ => b"ELRS: unknown receiver error",
            };
            // Error text shares the stream payload; it must never be forwarded to UART.
            out[43] = 0;
            out[44..44 + text.len()].copy_from_slice(text);
            out[251] = text.len() as u8;
        }
        if self.binding && self.phase != 5 && self.error != 1 {
            out[3] |= 1;
        }
        if self.version_ok {
            out[3] |= 64;
        }
        if let Some(uid) = self.bound {
            out[3] |= 2;
            out[106..112].copy_from_slice(&uid);
        }
        let rate = self.tracker.rate();
        put32(out, 8, self.tracker.frequency());
        put32(out, 12, uid_seed(&self.uid));
        out[16..18].copy_from_slice(&crc_initializer(&self.uid).to_le_bytes());
        out[18] = rate.size as u8;
        out[19] = u8::from(rate.flrc);
        out[20] = rate.sf << 4;
        out[21] = rate.coding;
        out[22] = rate.preamble;
        out[23] = u8::from(self.binding || self.uid[5] & 1 != 0);
        let mut pwm = [0; 16];
        let (count, valid) = if self.phase == 3 && self.error == 0 && !self.binding {
            self.tracker.read_channels(now, &mut pwm)
        } else {
            (0, 0)
        };
        out[112] = count;
        out[113] = ((u16::from(self.tracker.lq()) * 255 + 50) / 100) as u8;
        put32(out, 116, self.tracker.frame_sequence);
        put32(out, 120, now.wrapping_add(valid));
        put32(out, 124, self.tracker.last_input());
        put32(out, 128, 500000);
        put32(out, 132, Tracker::TIMEOUT);
        for (i, value) in pwm.iter().enumerate() {
            out[136 + i * 2..138 + i * 2].copy_from_slice(&value.to_le_bytes());
        }
        put32(out, 168, self.tx_count);
        put32(out, 172, self.tx_done);
        put32(out, 176, self.tx_timeout);
        for (i, value) in self.tracker.stats.iter().enumerate() {
            put32(out, 180 + i * 4, *value);
        }
        put32(out, 244, self.tracker.generation);
        out[248] = rate.wire;
        out[249] = self.tracker.lq();
        out[250] = self.tracker.state;
        let led = if self.error != 0 {
            4
        } else if self.binding {
            3
        } else if self.tracker.state == 2
            && self.tracker.seen & 15 == 15
            && now.wrapping_sub(self.tracker.last_input()) <= Tracker::TIMEOUT
        {
            2
        } else {
            1
        };
        if self.led_state != led {
            self.led_state = led;
            self.led_at = now;
        }
        let ms = now.wrapping_sub(self.led_at) / 1000;
        let on = match led {
            2 => true,
            3 => ms % 1300 < 100 || (200..300).contains(&(ms % 1300)),
            4 => ms % 1200 < 200,
            _ => ms % 1000 < 500,
        };
        if on {
            out[3] |= 4;
        }
    }
}
#[inline(never)]
unsafe fn initialize(state: *mut u8, input: &[u8]) {
    state.cast::<Receiver>().write(Receiver::new(input));
}
/// # Safety
/// The host owns aligned 4096-byte state and disjoint 128/256-byte I/O buffers;
/// it serializes calls, initializes once per generation and pins module memory.
#[no_mangle]
pub unsafe extern "C" fn sx1280_receiver_v1(
    state: *mut u8,
    input: *const u8,
    output: *mut u8,
) -> u32 {
    let input = core::slice::from_raw_parts(input, 128);
    let out = core::slice::from_raw_parts_mut(output, 256);
    if input[0] > 2
        || (input[0] == 2 && (input[1] > 62 || input[2] > 62))
        || (input[0] == 1 && input[18] > 16)
    {
        return 0;
    }
    if input[0] == 0 {
        initialize(state, input);
    }
    let s = &mut *state.cast::<Receiver>();
    let now = u32_at(input, 4);
    if input[0] == 1 && input[3] == 4 && s.transmitting {
        s.tx_at = u32_at(input, 36);
    }
    // Return to RX immediately after TX-done; process the next slot on its
    // own wakeup rather than rebuilding channels/transport in this short gap.
    if input[0] == 1
        && input[1] == 2
        && input[2] != 0
        && (input[3] == 0 || input[3] == 4)
        && s.phase == 3
        && s.transmitting
    {
        s.transmitting = false;
        s.tx_done = s.tx_done.wrapping_add(1);
        out.copy_from_slice(&s.snapshot_cache);
        out[1] = 4;
        out[25] = 0;
        out[42] = 0;
        out[43] = 0;
        put32(out, 4, s.tracker.wait(now).min(5000));
        put32(out, 172, s.tx_done);
        return 1;
    }
    out.fill(0);
    put32(out, 4, 5000);
    match input[0] {
        1 => s.radio(input, out, now),
        2 => s.main(input, out, now),
        _ => {}
    }
    s.snapshot(out, now);
    s.snapshot_cache.copy_from_slice(out);
    1
}

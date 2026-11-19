// SPDX-License-Identifier: GPL-3.0-or-later
//! LED output backends selected by SHOW_LED0_TYPE, as the fork's
//! DroneShowLEDFactory and DroneShowLED_* classes.

use ap_rust_sdk::enums::{MavSeverity, SrvChannelFunction};
use ap_rust_sdk::{Context, I2cDevice, Result, SerialPort};

use crate::lights_out::{Frame, LedOutput};
use crate::params::{Params, P};
use crate::show::lights::Rgbw;

const MSG_DEBUG_VECT: u32 = 250;

// SERVOn_FUNCTION scripting13..16 = W, R, G, B
const FN_WHITE: i32 = 106;
const FN_RED: i32 = 107;

enum Backend {
    None,
    Mavlink(u8),
    SerialLed { chan: u8, rgbw_virtual: Option<u8> },
    Debug { last_ms: u32 },
    Servo(Servo),
    I2c { dev: I2cDevice, rgbw: bool },
    WgDrones(SerialPort),
    Notify,
}

struct Servo {
    // R, G, B, W output channels (0-based)
    chan: [u8; 4],
    white: bool,
    inverted: bool,
    // SERVOn_MIN/MAX per channel when using limits
    limits: Option<[(u16, u16); 4]>,
    off_is_zero: bool,
    period_us: u32,
}

impl Servo {
    fn duty(&self, i: usize, c: u8) -> u16 {
        match self.limits {
            Some(l) => {
                let (min, max) = (l[i].0 as i32, l[i].1 as i32);
                if self.off_is_zero && c == 0 {
                    0
                } else {
                    let c = if self.off_is_zero { c as i32 - 1 } else { c as i32 };
                    (min + (max - min) * c / 255) as u16
                }
            }
            // never 0 us: some drivers treat it as no signal
            None => (self.period_us * c as u32 / 255).clamp(1, 2450) as u16,
        }
    }
}

fn param(c: &Context<'_>, name: &str) -> Result<Option<f32>> {
    match c.parameter(name)? {
        Some(p) => p.get(),
        None => Ok(None),
    }
}

fn now_ms(c: &Context<'_>) -> u32 {
    (c.time_us() / 1000) as u32
}

fn new_servo(c: &Context<'_>, led_type: i32) -> Result<Option<Servo>> {
    let srv = c.srv_channels();
    let mut chan = [255u8; 4];
    for k in 0..3 {
        match srv.find_channel(SrvChannelFunction(FN_RED + k as i32))? {
            Some(ch) => chan[k] = ch,
            None => return Ok(None),
        }
    }
    let white = match srv.find_channel(SrvChannelFunction(FN_WHITE))? {
        Some(ch) => {
            chan[3] = ch;
            true
        }
        None => false,
    };
    let limits = if led_type == 13 || led_type == 14 {
        let mut l = [(1000u16, 2000u16); 4];
        for k in 0..if white { 4 } else { 3 } {
            let mut buf = [0u8; 16];
            let min = crate::support::fmt(&mut buf, format_args!("SERVO{}_MIN", chan[k] + 1));
            l[k].0 = param(c, min)?.unwrap_or(1000.0) as u16;
            let mut buf = [0u8; 16];
            let max = crate::support::fmt(&mut buf, format_args!("SERVO{}_MAX", chan[k] + 1));
            l[k].1 = param(c, max)?.unwrap_or(2000.0) as u16;
        }
        Some(l)
    } else {
        None
    };
    // the fork forces LED channels to channel 0's rate; assume RC_SPEED
    let hz = param(c, "RC_SPEED")?.unwrap_or(50.0).max(1.0);
    Ok(Some(Servo {
        chan,
        white,
        inverted: led_type == 8,
        limits,
        off_is_zero: led_type == 13,
        period_us: (1_000_000.0 / hz) as u32,
    }))
}

fn new_backend(c: &Context<'_>, led_type: i32, chan: u8, count: u8) -> Result<Backend> {
    let sl = c.serial_led();
    // SDK range errors (bad channel/count) mean "no LED", as a failed init in the fork
    let ok = |r: Result<bool>| r.unwrap_or(false);
    Ok(match led_type {
        1 if chan < 4 => Backend::Mavlink(chan),
        2 if ok(sl.set_num_neopixel(chan, count)) => Backend::SerialLed { chan, rgbw_virtual: None },
        3 if ok(sl.set_num_profiled(chan, count)) => Backend::SerialLed { chan, rgbw_virtual: None },
        4 => Backend::Debug { last_ms: 0 },
        6 | 8 | 13 | 14 => new_servo(c, led_type)?.map_or(Backend::None, Backend::Servo),
        7 | 11 => match c.i2c().get_device(count, chan, 400_000, false) {
            Ok(Some(dev)) => Backend::I2c { dev, rgbw: led_type == 11 },
            _ => Backend::None,
        },
        9 => match c.serial().find_serial(0)? {
            Some(port) => {
                port.begin(c, None)?;
                Backend::WgDrones(port)
            }
            None => Backend::None,
        },
        10 => {
            // N RGBW LEDs emulated as ceil(4N/3) WS2812 LEDs
            let n = if count > 191 { 255 } else { ((count as u32 * 4 + 2) / 3) as u8 };
            if ok(sl.set_num_neopixel(chan, n)) {
                Backend::SerialLed { chan, rgbw_virtual: Some(n) }
            } else {
                Backend::None
            }
        }
        12 => Backend::Notify,
        _ => Backend::None,
    })
}

impl Backend {
    fn has_white(&self) -> bool {
        match self {
            Backend::SerialLed { rgbw_virtual, .. } => rgbw_virtual.is_some(),
            Backend::I2c { rgbw, .. } => *rgbw,
            Backend::Servo(s) => s.white,
            _ => false,
        }
    }

    /// Sends a gamma-corrected colour; false if it should be retried.
    fn send(&mut self, c: &Context<'_>, v: Rgbw) -> Result<bool> {
        match self {
            Backend::None => Ok(true),
            Backend::Mavlink(chan) => {
                let mut p = [0u8; 30];
                // the fork sends boot ms in time_usec
                p[0..8].copy_from_slice(&(now_ms(c) as u64).to_le_bytes());
                for (k, x) in [v.r, v.g, v.b].iter().enumerate() {
                    p[8 + 4 * k..12 + 4 * k].copy_from_slice(&(*x as f32 / 255.0).to_le_bytes());
                }
                p[20..23].copy_from_slice(b"rgb");
                c.mavlink().send_chan(*chan, MSG_DEBUG_VECT, &p, None)?;
                Ok(true)
            }
            Backend::SerialLed { chan, rgbw_virtual: None } => {
                let sl = c.serial_led();
                sl.set_rgb(*chan, -1, v.r, v.g, v.b)?;
                sl.send(*chan)?;
                Ok(true)
            }
            Backend::SerialLed { chan, rgbw_virtual: Some(n) } => {
                let (sl, chan, n) = (c.serial_led(), *chan, *n as usize);
                // the RGBW byte pattern repeats every 4 virtual LEDs; the last
                // virtual LED is masked so no extra LED lights up
                let groups = [[v.g, v.r, v.b], [v.r, v.w, v.g], [v.w, v.b, v.r], [v.b, v.g, v.w]];
                let mut i = 0;
                while i < n {
                    let k = i % 4;
                    let mut x = groups[k];
                    if i == n - 1 && (k == 1 || k == 2) {
                        x = if k == 1 { [0, v.w, 0] } else { [v.w, v.b, 0] };
                    }
                    sl.set_rgb(chan, i as i8, x[0], x[1], x[2])?;
                    i += 1;
                }
                sl.send(chan)?;
                Ok(true)
            }
            Backend::Debug { last_ms } => {
                // the fork never updated its timestamp; rate-limit for real
                let now = now_ms(c);
                if *last_ms != 0 && now.wrapping_sub(*last_ms) < 100 {
                    return Ok(false);
                }
                *last_ms = now.max(1);
                let mut buf = [0u8; 8];
                let s = crate::support::fmt(&mut buf, format_args!("{:02X}{:02X}{:02X}{:02X}", v.r, v.g, v.b, v.w));
                c.gcs().send_text(MavSeverity::INFO, s)?;
                Ok(true)
            }
            Backend::Servo(s) => {
                let x = [v.r, v.g, v.b, v.w];
                let srv = c.srv_channels();
                for k in 0..if s.white { 4 } else { 3 } {
                    let col = if s.inverted { 255 - x[k] } else { x[k] };
                    srv.set_output_pwm_chan(s.chan[k], s.duty(k, col))?;
                }
                Ok(true)
            }
            Backend::I2c { dev, rgbw } => {
                let buf = [v.r, v.g, v.b, v.w];
                dev.transfer(c, &buf[..if *rgbw { 4 } else { 3 }], &mut [])?;
                Ok(true)
            }
            Backend::WgDrones(port) => {
                let mut buf = [0u8; 32];
                let s = crate::support::fmt(&mut buf, format_args!("$c1@{}\n$c2@{}\n$c3@{}\n", v.r, v.g, v.b));
                port.write(c, s.as_bytes())?;
                Ok(true)
            }
            Backend::Notify => {
                c.notify().handle_rgb(v.r, v.g, v.b, 0)?;
                Ok(true)
            }
        }
    }

    fn close(self, c: &Context<'_>) {
        if let Backend::I2c { dev, .. } = self {
            let _ = dev.close(c);
        }
    }
}

/// The SHOW_LED0 light: backend plus output stage.
pub struct Led {
    key: Option<(i32, u8, u8)>,
    backend: Backend,
    out: LedOutput,
}

impl Default for Led {
    fn default() -> Self {
        Led { key: None, backend: Backend::None, out: LedOutput::default() }
    }
}

impl Led {
    /// Rebuilds the backend only if type, channel or count changed; updates
    /// gamma and minimum brightness. Call at init and on show mode entry/exit.
    pub fn configure(&mut self, c: &Context<'_>, p: &Params<'_, '_>) -> Result {
        let key = (p.get_i32(P::Led0Type), p.get_i32(P::Led0Chan) as u8, p.get_i32(P::Led0Count) as u8);
        if self.key != Some(key) {
            // turn the old LED off
            self.out.set_rgbw(Rgbw::default());
            let _ = self.flush(c);
            core::mem::replace(&mut self.backend, Backend::None).close(c);
            self.backend = new_backend(c, key.0, key.1, key.2)?;
            self.key = Some(key);
            self.out = LedOutput::default();
        }
        self.out.set_gamma(p.get(P::Led0Gamma));
        self.out.set_min_brightness(p.get(P::Led0Minbri));
        Ok(())
    }

    /// Outputs this tick's colour (even 50 Hz ticks).
    pub fn write(&mut self, c: &Context<'_>, f: Frame, p: &Params<'_, '_>) -> Result {
        self.out.set_gamma(p.get(P::Led0Gamma));
        self.out.set_frame(f, self.backend.has_white(), p.get(P::Led0Wtemp));
        self.flush(c)
    }

    /// Retries an unsent command (odd 50 Hz ticks).
    pub fn repeat(&mut self, c: &Context<'_>) -> Result {
        self.flush(c)
    }

    fn flush(&mut self, c: &Context<'_>) -> Result {
        if let Some(v) = self.out.pending() {
            if self.backend.send(c, v)? {
                self.out.sent();
            }
        }
        Ok(())
    }
}

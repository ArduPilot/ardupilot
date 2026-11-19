// SPDX-License-Identifier: GPL-3.0-or-later
//! Single-aircraft do_path equations. Vehicle ownership/abort handling lives in the runner.

use super::{
    math::*,
    path::{Attributes, Evaluation, Path},
};
use ap_rust_sdk::{Context, Error, Quaternion as Q, Result, Vector3f as V};

pub const LOOP_RATE: f32 = 40.0;

#[derive(Clone, Copy)]
pub struct Configuration {
    pub ang_accel: f32,
    pub ang_tc: f32,
    pub thr_pit_ff: f32,
    pub spd_p: f32,
    pub spd_i: f32,
    pub roll_corr_tc: f32,
    pub time_corr_p: f32,
    pub err_corr_p: f32,
    pub err_corr_d: f32,
    pub entry_rate: f32,
    pub thr_lookahead: f32,
    pub thr_min: f32,
    pub thr_boost: f32,
    pub yaw_accel: f32,
    pub lookahead: f32,
    pub path_scale: f32,
    pub ke_rudd: f32,
    pub ke_rudd_lookahead: f32,
    pub alt_abort: f32,
    pub airspeed_min: f32,
    pub scaling_speed: f32,
    pub airspeed_cruise: f32,
}

impl Configuration {
    pub fn validate(self) -> Result {
        let values = [
            self.ang_accel,
            self.ang_tc,
            self.thr_pit_ff,
            self.spd_p,
            self.spd_i,
            self.roll_corr_tc,
            self.time_corr_p,
            self.err_corr_p,
            self.err_corr_d,
            self.entry_rate,
            self.thr_lookahead,
            self.thr_min,
            self.thr_boost,
            self.yaw_accel,
            self.lookahead,
            self.path_scale,
            self.ke_rudd,
            self.ke_rudd_lookahead,
            self.alt_abort,
            self.airspeed_min,
            self.scaling_speed,
            self.airspeed_cruise,
        ];
        if values.iter().any(|v| !v.is_finite())
            || self.roll_corr_tc <= 0.0
            || self.ang_tc < 0.0
            || self.entry_rate <= 0.0
            || self.path_scale == 0.0
            || self.airspeed_min <= 0.0
            || self.scaling_speed <= 0.0
            || self.airspeed_cruise <= 0.0
            || !(0.0..=100.0).contains(&self.thr_min)
            || !(0.0..=100.0).contains(&self.thr_boost)
        {
            return Err(Error::Invalid);
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub struct Input {
    pub now: f32,
    pub pos: V,
    pub gyro: V,
    pub velocity: V,
    pub airspeed: f32,
    pub eas2tas: f32,
    pub attitude: Q,
    pub throttle: f32,
}

impl Input {
    pub fn validate(self) -> Result {
        let q = self.attitude;
        let norm = q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3 + q.q4 * q.q4;
        if !self.now.is_finite()
            || !finite(self.pos)
            || !finite(self.gyro)
            || !finite(self.velocity)
            || !self.airspeed.is_finite()
            || self.airspeed < 0.0
            || !self.eas2tas.is_finite()
            || self.eas2tas <= 0.0
            || !self.throttle.is_finite()
            || !norm.is_finite()
            || (norm - 1.0).abs() > 0.01
        {
            return Err(Error::Invalid);
        }
        Ok(())
    }
}

pub struct Output {
    pub throttle: f32,
    pub rates: V,
    pub rudder: f32,
    pub position_error: f32,
    pub path_rate_fallback: bool,
    pub lookahead_fallback: bool,
    pub track_pos: V,
    pub track_attitude: Q,
    pub measured_attitude: Q,
    pub path_t: f32,
    pub path_error_t: f32,
    pub q_change_t: f32,
    pub dt: f32,
    pub correction_rates: V,
    pub path_rates: V,
    pub roll_error_rate: f32,
    pub angle_error: f32,
    pub lookahead_rates: Option<(f32, f32, f32, f32)>,
    pub speed_log: [f32; 6],
}

pub enum Step {
    Waiting,
    Command(Output),
    // Lua applies this last command before releasing scripting control.
    AltitudeAbort(Output),
    Finished,
}

struct SpeedPI {
    p: f32,
    i: f32,
    integral: f32,
    total: f32,
    last_time: f32,
}
impl SpeedPI {
    fn update(
        &mut self,
        c: &Context<'_>,
        now: f32,
        target: f32,
        current: f32,
        pitch: f32,
        ff: f32,
    ) -> Result<(f32, [f32; 6])> {
        let dt = now - self.last_time;
        self.last_time = now;
        let error = target - current;
        let p = self.p * error;
        if (self.total < 100.0 && self.total > 0.0)
            || (self.total >= 100.0 && error < 0.0)
            || (self.total <= 0.0 && error > 0.0)
        {
            self.integral += self.i * error * dt;
        }
        self.integral = constrain(self.integral, -100.0, 100.0);
        self.total = constrain(p + self.integral, 0.0, 100.0);
        let feedforward = c.libm().sinf(pitch)? * ff;
        Ok((
            self.total + feedforward,
            [target, current, p, self.integral, self.total, feedforward],
        ))
    }
}

pub struct Controller {
    pub count: u32,
    target_speed: f32,
    length: f32,
    total_time: f32,
    initial_orientation: Q,
    initial_pos: V,
    accumulated_orientation: Q,
    last_time: f32,
    last_rates: V,
    pub path_t: f32,
    pos: V,
    roll: f32,
    shift: V,
    last_shift_xy: bool,
    tangent: V,
    speed_pi: SpeedPI,
}

impl Controller {
    pub fn save(&self, words: &mut [u32]) {
        words[..36].copy_from_slice(&[
            self.count,
            self.target_speed.to_bits(),
            self.length.to_bits(),
            self.total_time.to_bits(),
            self.initial_orientation.q1.to_bits(),
            self.initial_orientation.q2.to_bits(),
            self.initial_orientation.q3.to_bits(),
            self.initial_orientation.q4.to_bits(),
            self.initial_pos.x.to_bits(),
            self.initial_pos.y.to_bits(),
            self.initial_pos.z.to_bits(),
            self.accumulated_orientation.q1.to_bits(),
            self.accumulated_orientation.q2.to_bits(),
            self.accumulated_orientation.q3.to_bits(),
            self.accumulated_orientation.q4.to_bits(),
            self.last_time.to_bits(),
            self.last_rates.x.to_bits(),
            self.last_rates.y.to_bits(),
            self.last_rates.z.to_bits(),
            self.path_t.to_bits(),
            self.pos.x.to_bits(),
            self.pos.y.to_bits(),
            self.pos.z.to_bits(),
            self.roll.to_bits(),
            self.shift.x.to_bits(),
            self.shift.y.to_bits(),
            self.shift.z.to_bits(),
            u32::from(self.last_shift_xy),
            self.tangent.x.to_bits(),
            self.tangent.y.to_bits(),
            self.tangent.z.to_bits(),
            self.speed_pi.p.to_bits(),
            self.speed_pi.i.to_bits(),
            self.speed_pi.integral.to_bits(),
            self.speed_pi.total.to_bits(),
            self.speed_pi.last_time.to_bits(),
        ]);
    }

    pub fn restore(words: &[u32]) -> Result<Self> {
        if words.len() < 36
            || words[27] > 1
            || words[..36]
                .iter()
                .enumerate()
                .any(|(i, v)| i != 0 && i != 27 && !f32::from_bits(*v).is_finite())
        {
            return Err(Error::Invalid);
        }
        let f = |i: usize| f32::from_bits(words[i]);
        if f(1) <= 0.0 || f(2) <= 0.0 || f(3) <= 0.0 {
            return Err(Error::Invalid);
        }
        let q = |i| Q {
            q1: f(i),
            q2: f(i + 1),
            q3: f(i + 2),
            q4: f(i + 3),
        };
        let v = |i| V {
            x: f(i),
            y: f(i + 1),
            z: f(i + 2),
        };
        for i in [4, 11] {
            let a = q(i);
            if (a.q1 * a.q1 + a.q2 * a.q2 + a.q3 * a.q3 + a.q4 * a.q4 - 1.0).abs() > 0.01 {
                return Err(Error::Invalid);
            }
        }
        Ok(Self {
            count: words[0],
            target_speed: f(1),
            length: f(2),
            total_time: f(3),
            initial_orientation: q(4),
            initial_pos: v(8),
            accumulated_orientation: q(11),
            last_time: f(15),
            last_rates: v(16),
            path_t: f(19),
            pos: v(20),
            roll: f(23),
            shift: v(24),
            last_shift_xy: words[27] != 0,
            tangent: v(28),
            speed_pi: SpeedPI {
                p: f(31),
                i: f(32),
                integral: f(33),
                total: f(34),
                last_time: f(35),
            },
        })
    }
}

fn finite(v: V) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}
fn nan(v: V) -> bool {
    v.x.is_nan() || v.y.is_nan() || v.z.is_nan()
}
fn normalize(c: &Context<'_>, q: &mut Q) -> Result {
    let length = q.length(c)?;
    if !length.is_finite() || length < f32::EPSILON {
        return Err(Error::Invalid);
    }
    q.normalize(c)
}
fn angular_velocity(c: &Context<'_>, v: V, dt: f32) -> Result<Q> {
    let half = 0.5 * dt;
    let (x, y, z) = (half * v.x, half * v.y, half * v.z);
    if !(1.0 + x * x + y * y + z * z).is_finite() {
        return Err(Error::Invalid);
    }
    let mut q = Q::default();
    q.from_angular_velocity(c, v, dt)?;
    Ok(q)
}
fn unit(c: &Context<'_>, mut v: V) -> Result<V> {
    let length = v.length(c)?;
    if !length.is_finite() || length <= 0.0 {
        return Err(Error::Invalid);
    }
    v.normalize(c)?;
    Ok(v)
}
fn ortho(c: &Context<'_>, x: V, unit: V) -> Result<V> {
    unit.cross(c, unit.cross(c, x)?)
}

pub fn target_speed(c: &Context<'_>, input: Input, p: Configuration) -> Result<f32> {
    Ok(max(
        input.eas2tas * p.airspeed_cruise,
        input.velocity.length(c)?,
    ))
}

impl Controller {
    pub fn start(
        c: &Context<'_>,
        path: &mut Path,
        input: Input,
        p: Configuration,
        yaw: f32,
        count: u32,
    ) -> Result<Self> {
        p.validate()?;
        input.validate()?;
        if !yaw.is_finite() {
            return Err(Error::Invalid);
        }
        let speed = target_speed(c, input, p)?;
        let length = path.length() * p.path_scale.abs();
        if !speed.is_finite() || speed <= 0.0 || !length.is_finite() || length <= 0.0 {
            return Err(Error::Invalid);
        }
        let mut orientation = orient(c, 0.0, 0.0, yaw)?;
        normalize(c, &mut orientation)?;
        path.restart();
        path.activate(c, speed)?;
        let mut state = Self {
            count: count.saturating_add(1),
            target_speed: speed,
            length,
            total_time: length / speed,
            initial_orientation: orientation,
            initial_pos: input.pos,
            accumulated_orientation: orientation,
            last_time: input.now - 1.0 / LOOP_RATE,
            last_rates: input.gyro.scale(c, deg(1.0))?,
            path_t: 0.0,
            pos: input.pos,
            roll: 0.0,
            shift: V::default(),
            last_shift_xy: false,
            tangent: V::default(),
            speed_pi: SpeedPI {
                p: p.spd_p,
                i: p.spd_i,
                integral: input.throttle,
                total: 0.0,
                last_time: input.now,
            },
        };
        let (point, _, _) =
            state.sample(c, path, 0.1 / (state.total_time * LOOP_RATE), input, p)?;
        state.tangent = point.sub(c, state.pos)?;
        Ok(state)
    }

    fn sample(
        &self,
        c: &Context<'_>,
        path: &mut Path,
        t: f32,
        input: Input,
        p: Configuration,
    ) -> Result<(V, f32, Attributes)> {
        let t = constrain(t, 0.0, 1.0);
        let eval = Evaluation {
            speed: target_speed(c, input, p)?,
            entry_rate: p.entry_rate,
            announce: true,
        };
        let mut point = path.position(c, t, eval)?;
        let mut angle = path.roll(c, t, self.total_time, eval)?;
        let mut correction = path.correction(c, t, eval)?;
        let attributes = path.attributes(c, t, eval)?;
        point = point.add(c, self.shift)?;
        point = earth_to_body(c, self.initial_orientation, point)?;
        point = point.scale(c, p.path_scale.abs())?;
        if p.path_scale < 0.0 {
            point.y = -point.y;
            correction = -correction;
            angle = -angle;
            let inverse = self.initial_orientation.inverse(c)?;
            point = body_to_earth(c, inverse.mul(c, inverse)?, point)?;
        }
        Ok((
            point.add(c, self.initial_pos)?,
            rad(angle + correction),
            attributes,
        ))
    }

    pub fn step(
        &mut self,
        c: &Context<'_>,
        path: &mut Path,
        input: Input,
        p: Configuration,
    ) -> Result<Step> {
        p.validate()?;
        input.validate()?;
        self.count = self.count.saturating_add(1);
        let attitude = input.attitude.inverse(c)?;
        let velocity_length = input.velocity.length(c)?;
        let dt = input.now - self.last_time;
        if dt < 0.0 {
            return Err(Error::Invalid);
        }
        if dt < 0.25 / LOOP_RATE {
            return Ok(Step::Waiting);
        }
        if !dt.is_finite() || !p.roll_corr_tc.is_finite() || p.roll_corr_tc <= 0.0 {
            return Err(Error::Invalid);
        }
        self.last_time = input.now;
        let local_dt = (1.0 / LOOP_RATE) / self.total_time;
        if self.path_t + local_dt > 1.0 {
            return Ok(Step::Finished);
        }
        let airspeed = max(p.airspeed_min, input.airspeed);
        let mut p0 = self.pos;
        let r0 = self.roll;
        let (mut p1, _, attributes) = self.sample(c, path, self.path_t + local_dt, input, p)?;
        let shift_xy = attributes.shift_xy.unwrap_or(false);
        if shift_xy && !self.last_shift_xy {
            let current = body_to_earth(c, self.initial_orientation, input.pos)?;
            let path_point = body_to_earth(c, self.initial_orientation, p1)?;
            let mut shift = current.sub(c, path_point)?;
            shift.z = 0.0;
            self.shift = self.shift.add(c, shift)?;
            p1 = p1.add(c, earth_to_body(c, self.initial_orientation, shift)?)?;
            p0.x = p1.x;
            p0.y = p1.y;
        }
        self.last_shift_xy = shift_xy;
        let tangent1 = self.tangent;
        let mut tangent2 = p1.sub(c, p0)?;
        let mut tv = unit(c, tangent2)?;
        let path_distance = input.velocity.dot(c, tv)? * dt;
        let path_delta = constrain(path_distance / self.length, 0.2 * local_dt, 4.0 * local_dt);
        (p1, _, _) = self.sample(
            c,
            path,
            constrain(self.path_t + path_delta, 0.0, 1.0),
            input,
            p,
        )?;
        let last_path_t = self.path_t;
        self.path_t += path_delta;
        tangent2 = p1.sub(c, p0)?;
        tv = unit(c, tangent2)?;
        let position_error = input.pos.sub(c, p1)?;
        let path_distance_error = tv.dot(c, position_error)?;
        let path_error_t = constrain(
            path_distance_error / self.length,
            -0.9 * path_delta,
            2.0 * path_delta,
        );
        self.path_t += p.time_corr_p * path_error_t;
        let (point, mut r1, attributes) =
            self.sample(c, path, constrain(self.path_t, 0.0, 1.0), input, p)?;
        p1 = point;
        tangent2 = p1.sub(c, p0)?;
        let q_change_t = (self.path_t - last_path_t) * self.total_time;
        r1 = self.roll + wrap_pi(c, r1 - self.roll)?;
        let alpha = lowpass_alpha(q_change_t, p.ang_tc);
        r1 = (1.0 - alpha) * self.roll + alpha * r1;
        r1 = wrap_pi(c, r1)?;
        self.tangent = tangent2;
        self.pos = p1;
        self.roll = r1;
        let b = ortho(c, position_error, tv)?;
        let b_dot = ortho(c, input.velocity, tv)?;
        let mut acceleration = b
            .scale(c, p.err_corr_p)?
            .add(c, b_dot.scale(c, p.err_corr_d)?)?;
        if let Some(gain) = attributes.pos_gain_mul {
            acceleration = acceleration.scale(c, gain)?;
        }
        acceleration = acceleration.scale(c, attributes.pos_corr.unwrap_or(1.0))?;
        let acc_body = earth_to_body(c, attitude, acceleration)?;
        let tas = constrain(input.eas2tas * airspeed, 3.0, 100.0);
        let mut correction = V::new(0.0, -acc_body.z / tas, acc_body.y / tas);
        if nan(correction) {
            correction = V::default();
        }
        correction = correction.scale(c, deg(1.0))?;
        if self.count < 2 {
            correction = V::default();
        }
        // The Lua fallback replaces NaN tangent rates with zero. Local
        // Invalid prevents AP_Math's zero-quaternion internal-error flag.
        let (path_rate, path_rate_fallback) = match tangents_to_rate(c, tangent1, tangent2, dt) {
            Ok(v) if finite(v) => (v, false),
            Ok(_) | Err(Error::Invalid) => (V::default(), true),
            Err(e) => return Err(e),
        };
        let mut rate_earth = path_rate.scale(c, deg(1.0))?;
        if self.count < 3 {
            rate_earth.z = 0.0;
        }
        let mut rate_body = earth_to_body(c, attitude, rate_earth)?;
        rate_body.x = deg(wrap_pi(c, r1 - r0)? / dt);
        let delta = angular_velocity(c, path_rate, dt)?;
        self.accumulated_orientation = delta.mul(c, self.accumulated_orientation)?;
        normalize(c, &mut self.accumulated_orientation)?;
        let axis = earth_to_body(c, self.accumulated_orientation, V::new(1.0, 0.0, 0.0))?;
        let mut rolled = Q::default();
        rolled.from_axis_angle(c, axis, r1)?;
        let desired = rolled.mul(c, self.accumulated_orientation)?;
        let mut roll_error = desired.mul(c, attitude)?;
        normalize(c, &mut roll_error)?;
        let mut axis_angle = V::default();
        roll_error.to_axis_angle(c, &mut axis_angle)?;
        let mut angle = axis_angle.length(c)?;
        let error_axis = if angle < 0.00001 {
            angle = 0.0;
            V::new(1.0, 0.0, 0.0)
        } else {
            axis_angle.scale(c, 1.0 / angle)?
        };
        let error_rate = error_axis.scale(c, angle / p.roll_corr_tc)?;
        let mut error_body = earth_to_body(c, attitude, error_rate)?.scale(c, deg(1.0))?;
        error_body.y = 0.0;
        error_body.z = 0.0;
        let mut lookahead_rates = None;
        let mut lookahead_fallback = false;
        if p.lookahead > 0.0 {
            let (p2, _, _) = self.sample(
                c,
                path,
                constrain(self.path_t + p.lookahead / self.total_time, 0.0, 1.0),
                input,
                p,
            )?;
            let tangent3 = p2.sub(c, p1)?;
            match tangents_to_rate(c, tangent2, tangent3, 0.5 * (p.lookahead + 1.0 / LOOP_RATE)) {
                Ok(rate) => {
                    let ratio = velocity_length / self.target_speed;
                    let rate = rate.scale(c, ratio * ratio)?;
                    let ahead = earth_to_body(c, attitude, rate)?.scale(c, deg(1.0))?;
                    lookahead_rates = Some((rate_body.y, ahead.y, rate_body.z, ahead.z));
                    if finite(ahead) {
                        rate_body.y = ahead.y;
                        rate_body.z = ahead.z;
                    } else {
                        lookahead_fallback = true;
                    }
                }
                Err(Error::Invalid) => lookahead_fallback = true,
                Err(e) => return Err(e),
            }
        }
        let total = rate_body
            .add(c, correction)?
            .add(c, error_body)?
            .add(c, V::default())?;
        let mut difference = total.sub(c, self.last_rates)?;
        let max_delta = p.ang_accel * dt;
        let mut max_yaw = max_delta;
        if p.yaw_accel > 0.0 && (p.yaw_accel < p.ang_accel || p.ang_accel <= 0.0) {
            max_yaw = p.yaw_accel * dt;
        }
        if max_delta > 0.0 {
            difference.x = constrain(difference.x, -max_delta, max_delta);
            difference.y = constrain(difference.y, -max_delta, max_delta);
        }
        if max_yaw > 0.0 {
            difference.z = constrain(difference.z, -max_yaw, max_yaw);
        }
        let rates = self.last_rates.add(c, difference)?;
        self.last_rates = rates;
        let mut rudder = 0.0;
        if p.ke_rudd > 0.0 {
            let change = angular_velocity(c, input.gyro, -p.ke_rudd_lookahead)?;
            let anticipated = change.mul(c, attitude)?;
            let scale = p.scaling_speed / airspeed;
            rudder = constrain(
                -dcm_c_y(anticipated.inverse(c)?) * p.ke_rudd * (scale * scale),
                -100.0,
                100.0,
            );
        }
        let change = angular_velocity(c, path_rate, p.thr_lookahead)?;
        let anticipated = change.mul(c, desired)?;
        let pitch = max(anticipated.get_euler_pitch(c)?, desired.get_euler_pitch(c)?);
        let (throttle, speed_log) = self.speed_pi.update(
            c,
            input.now,
            self.target_speed,
            velocity_length,
            pitch,
            p.thr_pit_ff,
        )?;
        let mut minimum = p.thr_min;
        if attributes.thr_boost.unwrap_or(false) {
            minimum = max(minimum, p.thr_boost);
        }
        let throttle = constrain(throttle, minimum, 100.0);
        if !throttle.is_finite() || !finite(rates) || !rudder.is_finite() {
            return Err(Error::Invalid);
        }
        let output = Output {
            throttle,
            rates,
            rudder,
            position_error: position_error.length(c)?,
            path_rate_fallback,
            lookahead_fallback,
            track_pos: p1,
            track_attitude: desired,
            measured_attitude: input.attitude,
            path_t: self.path_t,
            path_error_t,
            q_change_t,
            dt,
            correction_rates: correction,
            path_rates: rate_body,
            roll_error_rate: error_body.x,
            angle_error: wrap_180(c, deg(angle))?,
            lookahead_rates,
            speed_log,
        };
        if input.pos.sub(c, self.initial_pos)?.z > p.alt_abort {
            Ok(Step::AltitudeAbort(output))
        } else {
            Ok(Step::Command(output))
        }
    }
}

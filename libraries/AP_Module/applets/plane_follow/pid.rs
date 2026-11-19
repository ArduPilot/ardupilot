// SPDX-License-Identifier: GPL-3.0-or-later

use ap_rust_sdk::{Context, LogValue, Result};

fn constrain(mut value: f32, minimum: f32, maximum: f32) -> f32 {
    if value < minimum {
        value = minimum;
    }
    if value > maximum {
        value = maximum;
    }
    value
}

fn lowpass_alpha(dt: f32, cutoff: f32) -> f32 {
    if dt <= 0.0 || cutoff <= 0.0 {
        return 1.0;
    }
    if cutoff.abs() < f32::EPSILON {
        return 1.0;
    }
    if dt.abs() < f32::EPSILON {
        return 0.0;
    }
    let rc = 1.0 / (core::f32::consts::PI * 2.0 * cutoff);
    dt / (dt + rc)
}

pub struct SpeedController {
    kp: f32,
    ki: f32,
    kd: f32,
    i_max: f32,
    minimum: f32,
    maximum: f32,
    last_us: Option<u64>,
    error: f32,
    derivative: f32,
    reset_filter: bool,
    i_term: f32,
    p_term: f32,
    d_term: f32,
    total: f32,
    target: f32,
    current: f32,
}

impl SpeedController {
    pub fn new(kp: f32, ki: f32, kd: f32, i_max: f32, minimum: f32, maximum: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            i_max,
            minimum,
            maximum,
            last_us: None,
            error: 0.0,
            derivative: 0.0,
            reset_filter: true,
            i_term: 0.0,
            p_term: 0.0,
            d_term: 0.0,
            total: 0.0,
            target: 0.0,
            current: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.i_term = 0.0;
        self.reset_filter = true;
    }

    pub fn update(
        &mut self,
        context: &Context<'_>,
        speed_current: f32,
        speed_error: f32,
    ) -> Result<f32> {
        let target = speed_current + speed_error;
        let now_us = context.time_us();
        let last_us = self.last_us.unwrap_or(now_us);
        let dt = now_us.saturating_sub(last_us) as f32 * 0.000_001;
        self.last_us = Some(now_us);
        let error = target - speed_current;

        if self.reset_filter {
            self.reset_filter = false;
            self.error = self.target - speed_current;
            self.derivative = 0.0;
        } else {
            let error_last = self.error;
            self.error += lowpass_alpha(dt, 0.01) * ((self.target - speed_current) - self.error);
            if dt >= 0.0 {
                let derivative = (self.error - error_last) / dt;
                self.derivative += lowpass_alpha(dt, 0.005) * (derivative - self.derivative);
            }
        }

        self.d_term = self.derivative * self.kd;
        self.p_term = self.kp * error;
        if (self.total < self.maximum && self.total > self.minimum)
            || (self.total >= self.maximum && error < 0.0)
            || (self.total <= self.minimum && error > 0.0)
        {
            self.i_term += self.ki * error * dt;
        }
        self.i_term = constrain(self.i_term, -self.i_max, self.i_max);
        let result = constrain(
            target + self.p_term + self.i_term + self.d_term,
            self.minimum,
            self.maximum,
        );
        self.target = target;
        self.current = speed_current;
        self.total = result;

        context.logger().write(
            "ZSPI",
            "Targ,Curr,P,I,D,Total,Add",
            None,
            &[
                LogValue::Float(self.target),
                LogValue::Float(self.current),
                LogValue::Float(self.p_term),
                LogValue::Float(self.i_term),
                LogValue::Float(self.d_term),
                LogValue::Float(self.total),
                LogValue::Float(0.0),
            ],
        )?;
        Ok(result)
    }
}

pub struct CrossTrackController {
    kp: f32,
    ki: f32,
    kd: f32,
    max_correction: f32,
    integral_limit: f32,
    integral: f32,
    last_error: f32,
}

impl CrossTrackController {
    pub fn new(kp: f32, ki: f32, kd: f32, max_correction: f32, integral_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            max_correction,
            integral_limit,
            integral: 0.0,
            last_error: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
    }

    pub fn compute(&mut self, desired_heading: f32, cross_track_error: f32, dt: f32) -> f32 {
        let error_rate = (cross_track_error - self.last_error) / dt;
        self.integral = constrain(
            self.integral + cross_track_error * dt,
            -self.integral_limit,
            self.integral_limit,
        );
        let correction = constrain(
            self.kp * cross_track_error + self.kd * error_rate + self.ki * self.integral,
            -self.max_correction,
            self.max_correction,
        );
        let mut corrected_heading = (desired_heading - correction) % 360.0;
        if corrected_heading < 0.0 {
            corrected_heading += 360.0;
        }
        self.last_error = cross_track_error;
        corrected_heading
    }
}

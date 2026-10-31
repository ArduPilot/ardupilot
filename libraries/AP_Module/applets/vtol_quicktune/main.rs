// SPDX-License-Identifier: GPL-3.0-or-later
//! Native port of AP_Scripting/applets/VTOL-quicktune.lua.
#![no_std]
#![no_main]

extern crate alloc;

mod parameters;
mod support;

use ap_rust_sdk::{
    enums::RcChannelAuxFunc, Context, Error, LogValue, Result,
};
use parameters::{
    Parameters, ANGLE_MAX, AUTO_FILTER, AUTO_SAVE, AXES, AXIS_NAMES, D, DOUBLE_TIME, FF, FLTD,
    FLTE, FLTT, GAIN_MARGIN, I, MAX_REDUCE, OPTIONS, OSC_SMAX, P, RC_FUNC, RP_PI_RATIO, SMAX,
    SUFFIX_NAMES, YAW_D_MAX, YAW_P_MAX, Y_PI_RATIO,
};
use support::{message, retry};

ap_rust_sdk::module!(run);

const UPDATE_RATE_HZ: f32 = 40.0;
const UPDATE_PERIOD_MS: u32 = 25;
const STAGE_DELAY: f32 = 4.0;
const PILOT_INPUT_DELAY: f32 = 4.0;
const YAW_FLTE_MAX: f32 = 8.0;
const FLTD_MUL: f32 = 0.5;
const FLTT_MUL: f32 = 0.5;
const DEFAULT_SMAX: f32 = 50.0;
const OPTION_TWO_POSITION: i32 = 1;

#[derive(Clone, Copy, Eq, PartialEq)]
enum Stage {
    D,
    P,
}

impl Stage {
    fn suffix(self) -> usize {
        match self {
            Self::D => D,
            Self::P => P,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Self::D => "D",
            Self::P => "P",
        }
    }
}

struct Slew {
    axis: usize,
    suffix: usize,
    delta: f32,
    steps: u8,
}

struct Tuner<'ctx, 'host> {
    parameters: Parameters<'ctx, 'host>,
    axes_done: [bool; 3],
    filters_done: [bool; 3],
    stage: Stage,
    last_stage_change: f32,
    last_gain_report: f32,
    last_pilot_input: f32,
    last_warning: f32,
    tune_done_time: Option<f32>,
    slew: Option<Slew>,
    need_restore: bool,
    aborted: bool,
}

fn now(context: &Context<'_>) -> f32 {
    (context.time_us() / 1000) as u32 as f32 * 0.001
}

impl<'ctx, 'host> Tuner<'ctx, 'host> {
    fn new(context: &'ctx Context<'host>, parameters: Parameters<'ctx, 'host>) -> Self {
        let time = now(context);
        Self {
            parameters,
            axes_done: [false; 3],
            filters_done: [false; 3],
            stage: Stage::D,
            last_stage_change: time,
            last_gain_report: time,
            last_pilot_input: time,
            last_warning: time,
            tune_done_time: None,
            slew: None,
            need_restore: false,
            aborted: false,
        }
    }

    fn reset_axes(&mut self) {
        self.axes_done = [false; 3];
        self.filters_done = [false; 3];
        self.stage = Stage::D;
    }

    fn current_axis(&self) -> Result<Option<usize>> {
        let axes = self.parameters.quick(AXES)? as i32;
        Ok((0..AXIS_NAMES.len()).find(|&axis| axes & (1 << axis) != 0 && !self.axes_done[axis]))
    }

    fn have_pilot_input(&self, context: &Context<'_>) -> Result<bool> {
        for channel in self.parameters.rc_input {
            if retry(context, || channel.norm_input_dz(context))?.abs() > 0.0 {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn slew_rate(&self, context: &Context<'_>, axis: usize) -> Result<f32> {
        let rates = retry(context, || context.ac_attitude_control().get_rpy_srate())?;
        Ok(match axis {
            0 => rates.0,
            1 => rates.1,
            2 => rates.2,
            _ => return Err(Error::Invalid),
        })
    }

    fn adjust_gain(&mut self, axis: usize, suffix: usize, value: f32) -> Result {
        if !value.is_finite() {
            return Err(Error::Invalid);
        }
        self.need_restore = true;
        self.parameters.set_gain(axis, suffix, value)?;
        if suffix == P && self.parameters.gain(axis, FF)? <= 0.0 {
            let ratio = self.parameters.quick(if axis == 2 { Y_PI_RATIO } else { RP_PI_RATIO })?;
            if ratio >= 1.0 {
                self.parameters.set_gain(axis, I, value / ratio)?;
            }
        }
        Ok(())
    }

    fn limit_gain(&self, context: &Context<'_>, axis: usize, suffix: usize, value: f32) -> Result<f32> {
        let saved = self.parameters.gains[Parameters::gain_index(axis, suffix)].saved;
        let maximum = self.parameters.quick(MAX_REDUCE)?;
        if maximum >= 0.0 && maximum < 100.0 && saved > 0.0 {
            let reduction = 100.0 * (saved - value) / saved;
            if reduction > maximum {
                let limited = saved * (100.0 - maximum) * 0.01;
                message(
                    context,
                    6,
                    format_args!(
                        "limiting {}_{} {:.3} -> {:.3}",
                        AXIS_NAMES[axis], SUFFIX_NAMES[suffix], value, limited
                    ),
                )?;
                return Ok(limited);
            }
        }
        Ok(value)
    }

    fn adjust_gain_limited(
        &mut self,
        context: &Context<'_>,
        axis: usize,
        suffix: usize,
        value: f32,
    ) -> Result {
        let value = self.limit_gain(context, axis, suffix, value)?;
        self.adjust_gain(axis, suffix, value)
    }

    fn setup_smax(&mut self) -> Result {
        for axis in 0..AXIS_NAMES.len() {
            if self.parameters.gain(axis, SMAX)? <= 0.0 {
                self.adjust_gain(axis, SMAX, DEFAULT_SMAX)?;
            }
        }
        Ok(())
    }

    fn setup_filters(&mut self, axis: usize) -> Result {
        if self.parameters.quick(AUTO_FILTER)? <= 0.0 {
            self.filters_done[axis] = true;
            return Ok(());
        }
        let gyro_filter = self
            .parameters
            .gyro_filter
            .get()?
            .ok_or(Error::Unavailable)?;
        self.adjust_gain(axis, FLTT, gyro_filter * FLTT_MUL)?;
        self.adjust_gain(axis, FLTD, gyro_filter * FLTD_MUL)?;
        if axis == 2 {
            let flte = self.parameters.gain(axis, FLTE)?;
            if !(0.0..=YAW_FLTE_MAX).contains(&flte) {
                self.adjust_gain(axis, FLTE, YAW_FLTE_MAX)?;
            }
        }
        self.filters_done[axis] = true;
        Ok(())
    }

    fn gain_limit(&self, axis: usize, suffix: usize) -> Result<f32> {
        if axis != 2 {
            return Ok(0.0);
        }
        match suffix {
            P => self.parameters.quick(YAW_P_MAX),
            D => self.parameters.quick(YAW_D_MAX),
            _ => Ok(0.0),
        }
    }

    fn setup_slew(&mut self, context: &Context<'_>, axis: usize, suffix: usize, value: f32) -> Result {
        let target = self.limit_gain(context, axis, suffix, value)?;
        let steps = (UPDATE_RATE_HZ / 2.0) as u8;
        let delta = (target - self.parameters.gain(axis, suffix)?) / f32::from(steps);
        self.slew = Some(Slew {
            axis,
            suffix,
            delta,
            steps,
        });
        Ok(())
    }

    fn log_gain(&self, context: &Context<'_>, axis: usize, suffix: usize) -> Result {
        let mut label = [0u8; 4];
        label[..3].copy_from_slice(AXIS_NAMES[axis].as_bytes());
        label[3] = SUFFIX_NAMES[suffix].as_bytes()[0];
        context.logger().write(
            "QUIK",
            "SRate,Gain,Param",
            None,
            &[
                LogValue::Float(self.slew_rate(context, axis)?),
                LogValue::Float(self.parameters.gain(axis, suffix)?),
                LogValue::Text4(&label),
            ],
        )
    }

    fn update_slew(&mut self, context: &Context<'_>) -> Result {
        let Some(mut slew) = self.slew.take() else {
            return Ok(());
        };
        let value = self.parameters.gain(slew.axis, slew.suffix)? + slew.delta;
        self.adjust_gain(slew.axis, slew.suffix, value)?;
        self.log_gain(context, slew.axis, slew.suffix)?;
        slew.steps -= 1;
        if slew.steps == 0 {
            message(
                context,
                6,
                format_args!(
                    "{}_{} {:.4}",
                    AXIS_NAMES[slew.axis],
                    SUFFIX_NAMES[slew.suffix],
                    self.parameters.gain(slew.axis, slew.suffix)?
                ),
            )?;
            if self.current_axis()?.is_none() {
                message(context, 5, format_args!("Tuning: DONE"))?;
                self.tune_done_time = Some(now(context));
            }
        } else {
            self.slew = Some(slew);
        }
        Ok(())
    }

    fn advance_stage(&mut self, context: &Context<'_>, axis: usize) -> Result {
        if self.stage == Stage::D {
            self.stage = Stage::P;
        } else {
            self.axes_done[axis] = true;
            message(context, 5, format_args!("Tuning: {} done", AXIS_NAMES[axis]))?;
            self.stage = Stage::D;
        }
        Ok(())
    }

    fn update(&mut self, context: &Context<'_>) -> Result {
        if self.parameters.quick(parameters::ENABLE)? < 1.0 {
            return Ok(());
        }
        let time = now(context);
        if self.have_pilot_input(context)? {
            self.last_pilot_input = time;
        }

        let rc_function = self.parameters.quick(RC_FUNC)? as i32;
        let Some(switch) = retry(context, || {
            context.rc().get_aux_cached(RcChannelAuxFunc(rc_function))
        })? else {
            return Ok(());
        };
        let two_position = self.parameters.quick(OPTIONS)? as i32 & OPTION_TWO_POSITION != 0;
        let tune_position = if two_position { 2 } else { 1 };
        let save_position = if two_position { u8::MAX } else { 2 };

        if self.aborted {
            if switch == 0 {
                self.aborted = false;
            } else {
                return Ok(());
            }
        }
        let armed = retry(context, || context.arming().is_armed())?;
        let flying = retry(context, || context.vehicle().get_likely_flying())?;
        if switch == tune_position && (!armed || !flying) && time > self.last_warning + 5.0 {
            message(context, 0, format_args!("Tuning: Must be flying to tune"))?;
            self.last_warning = time;
            return Ok(());
        }
        if switch == 0 || !armed || !flying {
            if self.need_restore {
                self.parameters.restore()?;
                self.need_restore = false;
                message(context, 0, format_args!("Tuning: reverted"))?;
                self.tune_done_time = None;
                self.aborted = true;
            }
            self.reset_axes();
            return Ok(());
        }
        if switch == save_position && self.need_restore {
            self.parameters.save()?;
            self.need_restore = false;
            message(context, 5, format_args!("Tuning: saved"))?;
        }
        if switch != tune_position {
            return Ok(());
        }

        let maximum_angle = self.parameters.quick(ANGLE_MAX)?;
        if maximum_angle > 0.0 && self.need_restore {
            let attitude_error = retry(context, || {
                context.ac_attitude_control().get_att_error_angle_deg()
            })?;
            if attitude_error > maximum_angle {
                self.parameters.restore()?;
                self.need_restore = false;
                message(
                    context,
                    2,
                    format_args!(
                        "Tuning: attitude error {:.1}deg - ABORTING",
                        attitude_error
                    ),
                )?;
                self.tune_done_time = None;
                self.aborted = true;
                self.reset_axes();
                return Ok(());
            }
        }

        if time - self.last_stage_change < STAGE_DELAY {
            return self.update_slew(context);
        }
        let Some(axis) = self.current_axis()? else {
            if let Some(done) = self.tune_done_time {
                let autosave = self.parameters.quick(AUTO_SAVE)?;
                if autosave > 0.0 && time - done > autosave {
                    self.parameters.save()?;
                    self.need_restore = false;
                    message(context, 5, format_args!("Tuning: saved"))?;
                    self.tune_done_time = None;
                }
            }
            return Ok(());
        };

        if !self.need_restore {
            message(context, 5, format_args!("Tuning: starting tune"))?;
            self.parameters.refresh_saved()?;
            self.setup_smax()?;
        }
        if time - self.last_pilot_input < PILOT_INPUT_DELAY {
            return Ok(());
        }
        if !self.filters_done[axis] {
            message(context, 6, format_args!("Starting {} tune", AXIS_NAMES[axis]))?;
            self.setup_filters(axis)?;
        }

        let suffix = self.stage.suffix();
        let slew_rate = self.slew_rate(context, axis)?;
        let gain = self.parameters.gain(axis, suffix)?;
        let limit = self.gain_limit(axis, suffix)?;
        let oscillating = slew_rate > self.parameters.quick(OSC_SMAX)?;
        let limited = limit > 0.0 && gain >= limit;
        if limited || oscillating {
            let reduction = if oscillating {
                (100.0 - self.parameters.quick(GAIN_MARGIN)?) * 0.01
            } else {
                1.0
            };
            let mut new_gain = gain * reduction;
            if limit > 0.0 && new_gain > limit {
                new_gain = limit;
            }
            let old_gain = self.parameters.gains[Parameters::gain_index(axis, suffix)].saved;
            if new_gain < old_gain && suffix == D && axis != 2 {
                let ratio = (new_gain / old_gain).max(0.5);
                let old_p = self.parameters.gain(axis, P)?;
                let new_p = old_p * ratio;
                message(
                    context,
                    6,
                    format_args!(
                        "adjusting {}_P {:.3} -> {:.3}",
                        AXIS_NAMES[axis], old_p, new_p
                    ),
                )?;
                self.adjust_gain_limited(context, axis, P, new_p)?;
            }
            self.setup_slew(context, axis, suffix, new_gain)?;
            self.log_gain(context, axis, suffix)?;
            message(
                context,
                6,
                format_args!("Tuning: {}_{} done", AXIS_NAMES[axis], self.stage.name()),
            )?;
            self.advance_stage(context, axis)?;
            self.last_stage_change = time;
        } else {
            let multiplier = context.libm().expf(
                context.libm().logf(2.0)?
                    / (UPDATE_RATE_HZ * self.parameters.quick(DOUBLE_TIME)?),
            )?;
            let mut new_gain = gain * multiplier;
            if new_gain <= 0.0001 {
                new_gain = 0.001;
            }
            self.adjust_gain_limited(context, axis, suffix, new_gain)?;
            self.log_gain(context, axis, suffix)?;
            if time - self.last_gain_report > 3.0 {
                self.last_gain_report = time;
                message(
                    context,
                    6,
                    format_args!(
                        "{}_{} {:.4} sr:{:.2}",
                        AXIS_NAMES[axis], self.stage.name(), new_gain, slew_rate
                    ),
                )?;
            }
        }
        Ok(())
    }
}

fn run(context: &Context<'_>) -> Result {
    let (parameters, quadplane) = match Parameters::new(context) {
        Ok(value) => value,
        Err(error) => {
            message(context, 0, format_args!("Quicktune parameter error: {:?}", error))?;
            return Err(error);
        }
    };
    message(
        context,
        0,
        format_args!(
            "Quicktune for {} loaded",
            if quadplane { "quadplane" } else { "multicopter" }
        ),
    )?;
    let mut tuner = Tuner::new(context, parameters);
    loop {
        match tuner.update(context) {
            Ok(()) => context.sleep_ms(UPDATE_PERIOD_MS)?,
            Err(Error::Stopped) => return Err(Error::Stopped),
            Err(error) => {
                message(context, 0, format_args!("Internal Error: {:?}", error))?;
                context.sleep_ms(1000)?;
            }
        }
    }
}

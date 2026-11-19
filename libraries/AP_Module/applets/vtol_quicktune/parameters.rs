// SPDX-License-Identifier: GPL-3.0-or-later
//! Dynamic parameters and cached gain handles shared with the Lua applet.

extern crate alloc;

use super::support::{retry, Text};
use alloc::vec::Vec;
use ap_rust_sdk::{Context, Error, Parameter, RcChannel, Result};

ap_rust_sdk::param_table! {
    pub struct QuickTable, enum QuickParam;
    key = 8, prefix = "QUIK_";
    size = 15;

    // @Param: QUIK_ENABLE
    // @DisplayName: Quicktune enable
    // @Description: Enable quicktune system
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    Enable = 1, "ENABLE", f32: 0.0;

    // @Param: QUIK_AXES
    // @DisplayName: Quicktune axes
    // @Description: axes to tune
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    Axes = 2, "AXES", f32: 7.0;

    // @Param: QUIK_DOUBLE_TIME
    // @DisplayName: Quicktune doubling time
    // @Description: Time to double a tuning parameter. Raise this for a slower tune.
    // @Range: 5 20
    // @Units: s
    // @User: Standard
    DoubleTime = 3, "DOUBLE_TIME", f32: 10.0;

    // @Param: QUIK_GAIN_MARGIN
    // @DisplayName: Quicktune gain margin
    // @Description: Reduction in gain after oscillation detected. Raise this number to get a more conservative tune
    // @Range: 20 80
    // @Units: %
    // @User: Standard
    GainMargin = 4, "GAIN_MARGIN", f32: 60.0;

    // @Param: QUIK_OSC_SMAX
    // @DisplayName: Quicktune oscillation rate threshold
    // @Description: Threshold for oscillation detection. A lower value will lead to a more conservative tune.
    // @Range: 1 10
    // @User: Standard
    OscSmax = 5, "OSC_SMAX", f32: 5.0;

    // @Param: QUIK_YAW_P_MAX
    // @DisplayName: Quicktune Yaw P max
    // @Description: Maximum value for yaw P gain
    // @Range: 0.1 3
    // @User: Standard
    YawPMax = 6, "YAW_P_MAX", f32: 0.5;

    // @Param: QUIK_YAW_D_MAX
    // @DisplayName: Quicktune Yaw D max
    // @Description: Maximum value for yaw D gain
    // @Range: 0.001 1
    // @User: Standard
    YawDMax = 7, "YAW_D_MAX", f32: 0.01;

    // @Param: QUIK_RP_PI_RATIO
    // @DisplayName: Quicktune roll/pitch PI ratio
    // @Description: Ratio between P and I gains for roll and pitch. Raise this to get a lower I gain
    // @Range: 0.5 1.0
    // @User: Standard
    RpPiRatio = 8, "RP_PI_RATIO", f32: 1.0;

    // @Param: QUIK_Y_PI_RATIO
    // @DisplayName: Quicktune Yaw PI ratio
    // @Description: Ratio between P and I gains for yaw. Raise this to get a lower I gain
    // @Range: 0.5 20
    // @User: Standard
    YPiRatio = 9, "Y_PI_RATIO", f32: 10.0;

    // @Param: QUIK_AUTO_FILTER
    // @DisplayName: Quicktune auto filter enable
    // @Description: When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AutoFilter = 10, "AUTO_FILTER", f32: 1.0;

    // @Param: QUIK_AUTO_SAVE
    // @DisplayName: Quicktune auto save
    // @Description: Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune
    // @Units: s
    // @User: Standard
    AutoSave = 11, "AUTO_SAVE", f32: 0.0;

    // @Param: QUIK_RC_FUNC
    // @DisplayName: Quicktune RC function
    // @Description: RCn_OPTION number to use to control tuning stop/start/save
    // @User: Standard
    RcFunc = 12, "RC_FUNC", f32: 300.0;

    // @Param: QUIK_MAX_REDUCE
    // @DisplayName: Quicktune maximum gain reduction
    // @Description: This controls how much quicktune is allowed to lower gains from the original gains. If the vehicle already has a reasonable tune and is not oscillating then you can set this to zero to prevent gain reductions. The default of 20% is reasonable for most vehicles. Using a maximum gain reduction lowers the chance of an angle P oscillation happening if quicktune gets a false positive oscillation at a low gain, which can result in very low rate gains and a dangerous angle P oscillation.
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    MaxReduce = 13, "MAX_REDUCE", f32: 20.0;

    // @Param: QUIK_OPTIONS
    // @DisplayName: Quicktune options
    // @Description: Additional options. When the Two Position Switch option is enabled then a high switch position will start the tune, low will disable the tune. you should also set a QUIK_AUTO_SAVE time so that you will be able to save the tune.
    // @Bitmask: 0:UseTwoPositionSwitch
    // @User: Standard
    Options = 14, "OPTIONS", f32: 0.0;

    // @Param: QUIK_ANGLE_MAX
    // @DisplayName: maximum angle error for tune abort
    // @Description: If while tuning the angle error goes over this limit then the tune will aborts to prevent a bad oscillation in the case of the tuning algorithm failing. If you get an error "Tuning: attitude error ABORTING" and you think it is a false positive then you can either raise this parameter or you can try increasing the QUIK_DOUBLE_TIME to do the tune more slowly. A value of zero disables this check.
    // @Units: deg
    // @User: Standard
    AngleMax = 15, "ANGLE_MAX", f32: 10.0;
}

pub const ENABLE: usize = QuickParam::Enable as usize;
pub const AXES: usize = QuickParam::Axes as usize;
pub const DOUBLE_TIME: usize = QuickParam::DoubleTime as usize;
pub const GAIN_MARGIN: usize = QuickParam::GainMargin as usize;
pub const OSC_SMAX: usize = QuickParam::OscSmax as usize;
pub const YAW_P_MAX: usize = QuickParam::YawPMax as usize;
pub const YAW_D_MAX: usize = QuickParam::YawDMax as usize;
pub const RP_PI_RATIO: usize = QuickParam::RpPiRatio as usize;
pub const Y_PI_RATIO: usize = QuickParam::YPiRatio as usize;
pub const AUTO_FILTER: usize = QuickParam::AutoFilter as usize;
pub const AUTO_SAVE: usize = QuickParam::AutoSave as usize;
pub const RC_FUNC: usize = QuickParam::RcFunc as usize;
pub const MAX_REDUCE: usize = QuickParam::MaxReduce as usize;
pub const OPTIONS: usize = QuickParam::Options as usize;
pub const ANGLE_MAX: usize = QuickParam::AngleMax as usize;

pub const AXIS_NAMES: [&str; 3] = ["RLL", "PIT", "YAW"];
pub const SUFFIX_NAMES: [&str; 8] = ["FF", "P", "I", "D", "SMAX", "FLTT", "FLTD", "FLTE"];
pub const FF: usize = 0;
pub const P: usize = 1;
pub const I: usize = 2;
pub const D: usize = 3;
pub const SMAX: usize = 4;
pub const FLTT: usize = 5;
pub const FLTD: usize = 6;
pub const FLTE: usize = 7;

pub struct Gain<'ctx, 'host> {
    pub parameter: Parameter<'ctx, 'host>,
    pub saved: f32,
    pub changed: bool,
}

pub struct Parameters<'ctx, 'host> {
    quick: Vec<Parameter<'ctx, 'host>>,
    pub gains: Vec<Gain<'ctx, 'host>>,
    pub gyro_filter: Parameter<'ctx, 'host>,
    pub rc_input: [RcChannel; 3],
}

fn bind<'ctx, 'host>(context: &'ctx Context<'host>, name: &str) -> Result<Parameter<'ctx, 'host>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Unavailable)
}

fn read(parameter: &Parameter<'_, '_>) -> Result<f32> {
    let value = parameter.get()?.ok_or(Error::Unavailable)?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(Error::Invalid)
    }
}

fn set(parameter: &Parameter<'_, '_>, value: f32) -> Result {
    if parameter.set(value)? {
        Ok(())
    } else {
        Err(Error::Host)
    }
}

impl<'ctx, 'host> Parameters<'ctx, 'host> {
    pub fn new(context: &'ctx Context<'host>) -> Result<(Self, bool)> {
        QuickTable::register_definitions(context)?;
        let mut quick = Vec::new();
        quick.try_reserve_exact(QuickTable::definitions().len()).map_err(|_| Error::NoMemory)?;
        for d in QuickTable::definitions() {
            quick.push(d.bind(context, QuickTable::PREFIX)?);
        }

        let quadplane = retry(context, || context.param().get("Q_A_RAT_RLL_SMAX"))?.is_some();
        let prefix = if quadplane {
            "Q_A"
        } else if retry(context, || context.param().get("ATC_RAT_RLL_SMAX"))?.is_some() {
            "ATC"
        } else {
            return Err(Error::Unavailable);
        };

        let mut gains = Vec::new();
        gains
            .try_reserve_exact(AXIS_NAMES.len() * SUFFIX_NAMES.len())
            .map_err(|_| Error::NoMemory)?;
        for axis in AXIS_NAMES {
            for suffix in SUFFIX_NAMES {
                let name = Text::<17>::new(format_args!("{prefix}_RAT_{axis}_{suffix}"))?;
                let parameter = bind(context, name.as_str())?;
                let saved = read(&parameter)?;
                gains.push(Gain {
                    parameter,
                    saved,
                    changed: false,
                });
            }
        }

        let gyro_filter = bind(context, "INS_GYRO_FILTER")?;
        let mut channels = [None, None, None];
        for (slot, name) in channels.iter_mut().zip(["RCMAP_ROLL", "RCMAP_PITCH", "RCMAP_YAW"]) {
            let map = read(&bind(context, name)?)?;
            if map < 1.0 || map > u8::MAX as f32 || map != f32::from(map as u8) {
                return Err(Error::Invalid);
            }
            *slot = retry(context, || context.rc().get_channel(map as u8))?;
        }
        let rc_input = [
            channels[0].ok_or(Error::Unavailable)?,
            channels[1].ok_or(Error::Unavailable)?,
            channels[2].ok_or(Error::Unavailable)?,
        ];
        Ok((Self { quick, gains, gyro_filter, rc_input }, quadplane))
    }

    pub fn quick(&self, index: usize) -> Result<f32> {
        read(self.quick.get(index).ok_or(Error::Invalid)?)
    }

    pub fn gain_index(axis: usize, suffix: usize) -> usize {
        axis * SUFFIX_NAMES.len() + suffix
    }

    pub fn gain(&self, axis: usize, suffix: usize) -> Result<f32> {
        read(
            &self
                .gains
                .get(Self::gain_index(axis, suffix))
                .ok_or(Error::Invalid)?
                .parameter,
        )
    }

    pub fn set_gain(&mut self, axis: usize, suffix: usize, value: f32) -> Result {
        let gain = self
            .gains
            .get_mut(Self::gain_index(axis, suffix))
            .ok_or(Error::Invalid)?;
        set(&gain.parameter, value)?;
        gain.changed = true;
        Ok(())
    }

    pub fn refresh_saved(&mut self) -> Result {
        for gain in &mut self.gains {
            gain.saved = read(&gain.parameter)?;
        }
        Ok(())
    }

    pub fn restore(&mut self) -> Result {
        let mut first_error = None;
        for gain in &mut self.gains {
            if gain.changed {
                match set(&gain.parameter, gain.saved) {
                    Ok(()) => gain.changed = false,
                    Err(error) => {
                        first_error.get_or_insert(error);
                    }
                };
            }
        }
        first_error.map_or(Ok(()), Err)
    }

    pub fn save(&mut self) -> Result {
        let mut first_error = None;
        for gain in &mut self.gains {
            if gain.changed {
                let result = read(&gain.parameter).and_then(|value| {
                    if gain.parameter.set_and_save(value)? {
                        Ok(value)
                    } else {
                        Err(Error::Host)
                    }
                });
                match result {
                    Ok(value) => {
                        gain.saved = value;
                        gain.changed = false;
                    }
                    Err(error) => {
                        first_error.get_or_insert(error);
                    }
                }
            }
        }
        first_error.map_or(Ok(()), Err)
    }
}

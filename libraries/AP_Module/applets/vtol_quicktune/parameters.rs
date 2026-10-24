// SPDX-License-Identifier: GPL-3.0-or-later
//! Dynamic parameters and cached gain handles shared with the Lua applet.

extern crate alloc;

use super::support::{retry, Text};
use alloc::vec::Vec;
use ap_rust_sdk::{Context, Error, Parameter, RcChannel, Result};

const TABLE_KEY: u8 = 8;
const TABLE_PREFIX: &str = "QUIK_";
const QUICK_DEFINITIONS: &[(&str, f32)] = &[
    ("ENABLE", 0.0),
    ("AXES", 7.0),
    ("DOUBLE_TIME", 10.0),
    ("GAIN_MARGIN", 60.0),
    ("OSC_SMAX", 5.0),
    ("YAW_P_MAX", 0.5),
    ("YAW_D_MAX", 0.01),
    ("RP_PI_RATIO", 1.0),
    ("Y_PI_RATIO", 10.0),
    ("AUTO_FILTER", 1.0),
    ("AUTO_SAVE", 0.0),
    ("RC_FUNC", 300.0),
    ("MAX_REDUCE", 20.0),
    ("OPTIONS", 0.0),
    ("ANGLE_MAX", 10.0),
];

pub const ENABLE: usize = 0;
pub const AXES: usize = 1;
pub const DOUBLE_TIME: usize = 2;
pub const GAIN_MARGIN: usize = 3;
pub const OSC_SMAX: usize = 4;
pub const YAW_P_MAX: usize = 5;
pub const YAW_D_MAX: usize = 6;
pub const RP_PI_RATIO: usize = 7;
pub const Y_PI_RATIO: usize = 8;
pub const AUTO_FILTER: usize = 9;
pub const AUTO_SAVE: usize = 10;
pub const RC_FUNC: usize = 11;
pub const MAX_REDUCE: usize = 12;
pub const OPTIONS: usize = 13;
pub const ANGLE_MAX: usize = 14;

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
        if !retry(context, || context.param().add_table(TABLE_KEY, TABLE_PREFIX, 15))? {
            return Err(Error::Host);
        }
        let mut quick = Vec::new();
        quick
            .try_reserve_exact(QUICK_DEFINITIONS.len())
            .map_err(|_| Error::NoMemory)?;
        for (offset, &(name, default)) in QUICK_DEFINITIONS.iter().enumerate() {
            if !retry(context, || {
                context.param().add_param(TABLE_KEY, (offset + 1) as u8, name, default)
            })? {
                return Err(Error::Host);
            }
            let full_name = Text::<17>::new(format_args!("{TABLE_PREFIX}{name}"))?;
            quick.push(bind(context, full_name.as_str())?);
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

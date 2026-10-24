// SPDX-License-Identifier: GPL-3.0-or-later
//! Shared Lua parameter layout, with a native-module language selector.

use super::{
    controller::Configuration,
    math::constrain,
    support::{retry, Text},
};
use ap_rust_sdk::{Context, Error, Parameter, Result};

const AEROM: &[(u8, &str, f32)] = &[
    (1, "ANG_ACCEL", 6000.0),
    (2, "ANG_TC", 0.1),
    (4, "THR_PIT_FF", 80.0),
    (5, "SPD_P", 5.0),
    (6, "SPD_I", 25.0),
    (8, "ROL_COR_TC", 0.25),
    (11, "TIME_COR_P", 1.0),
    (12, "ERR_COR_P", 2.0),
    (13, "ERR_COR_D", 2.8),
    (14, "ENTRY_RATE", 60.0),
    (15, "THR_LKAHD", 1.0),
    (16, "DEBUG", 0.0),
    (17, "THR_MIN", 0.0),
    (18, "THR_BOOST", 50.0),
    (19, "YAW_ACCEL", 1500.0),
    (20, "LKAHD", 0.5),
    (21, "PATH_SCALE", 1.0),
    (22, "BOX_WIDTH", 400.0),
    (23, "STALL_THR", 40.0),
    (24, "STALL_PIT", -20.0),
    (26, "KE_RUDD", 25.0),
    (27, "KE_RUDD_LK", 0.25),
    (28, "ALT_ABORT", 15.0),
    (29, "TS_P", 0.33),
    (30, "TS_I", 0.33),
    (31, "TS_SPDMAX", 0.0),
    (32, "TS_RATE", 4.0),
    (33, "MIS_ANGLE", 0.0),
    (34, "OPTIONS", 0.0),
    (35, "LANG", 0.0),
];

pub struct Parameters<'a, 'h> {
    aerom: [Option<Parameter<'a, 'h>>; 41],
    pub trik: [Option<Parameter<'a, 'h>>; 64],
    airspeed_min: Parameter<'a, 'h>,
    scaling_speed: Parameter<'a, 'h>,
    airspeed_cruise: Parameter<'a, 'h>,
    // Lua captures speed PI gains at script startup, not on each update.
    spd_p: f32,
    spd_i: f32,
    pub trick_count: usize,
}

pub fn read(p: &Parameter<'_, '_>) -> Result<f32> {
    let value = p.get()?.ok_or(Error::Invalid)?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(Error::Invalid)
    }
}

fn bind<'a, 'h>(c: &'a Context<'h>, name: &str) -> Result<Parameter<'a, 'h>> {
    retry(c, || c.parameter(name))?.ok_or(Error::Invalid)
}
fn add<'a, 'h>(
    c: &'a Context<'h>,
    key: u8,
    prefix: &str,
    index: u8,
    name: &str,
    default: f32,
) -> Result<Parameter<'a, 'h>> {
    if !retry(c, || c.param().add_param(key, index, name, default))? {
        return Err(Error::Host);
    }
    bind(c, Text::<16>::new(format_args!("{prefix}{name}"))?.as_str())
}

impl<'a, 'h> Parameters<'a, 'h> {
    pub fn new(c: &'a Context<'h>) -> Result<Self> {
        if !retry(c, || c.param().add_table(70, "AEROM_", 40))?
            || !retry(c, || c.param().add_table(71, "TRIK", 63))?
        {
            return Err(Error::Host);
        }
        let mut aerom = core::array::from_fn(|_| None);
        for &(index, name, value) in AEROM {
            aerom[index as usize] = Some(add(c, 70, "AEROM_", index, name, value)?);
        }
        let mut trik = core::array::from_fn(|_| None);
        trik[1] = Some(add(c, 71, "TRIK", 1, "_ENABLE", 0.0)?);
        let mut result = Self {
            spd_p: read(aerom[5].as_ref().unwrap())?,
            spd_i: read(aerom[6].as_ref().unwrap())?,
            aerom,
            trik,
            airspeed_min: bind(c, "AIRSPEED_MIN")?,
            scaling_speed: bind(c, "SCALING_SPEED")?,
            airspeed_cruise: bind(c, "AIRSPEED_CRUISE")?,
            trick_count: 0,
        };
        let accel = result.a(1)?;
        if accel > 0.0 && accel < 100.0 {
            result.aerom[1].as_ref().unwrap().set_and_save(3000.0)?;
        }
        if result.a(2)? > 1.0 {
            result.aerom[2].as_ref().unwrap().set_and_save(0.2)?;
        }
        if result.t(1)? > 0.0 {
            for (index, name, default) in [
                (2, "_SEL_FN", 301.0),
                (3, "_ACT_FN", 300.0),
                (4, "_COUNT", 3.0),
            ] {
                result.trik[index as usize] = Some(add(c, 71, "TRIK", index, name, default)?);
            }
            result.trick_count = c.libm().floorf(constrain(result.t(4)?, 1.0, 11.0))? as usize;
            for i in 1..=result.trick_count {
                for (offset, suffix, default) in [
                    (0, "ID", -1.0),
                    (1, "ARG1", 30.0),
                    (2, "ARG2", 0.0),
                    (3, "ARG3", 0.0),
                    (4, "ARG4", 0.0),
                ] {
                    let index = 5 * i + offset;
                    let name = Text::<12>::new(format_args!("{i}_{suffix}"))?;
                    result.trik[index] =
                        Some(add(c, 71, "TRIK", index as u8, name.as_str(), default)?);
                }
            }
        }
        Ok(result)
    }

    pub fn a(&self, index: usize) -> Result<f32> {
        read(
            self.aerom
                .get(index)
                .and_then(Option::as_ref)
                .ok_or(Error::Invalid)?,
        )
    }
    pub fn t(&self, index: usize) -> Result<f32> {
        read(
            self.trik
                .get(index)
                .and_then(Option::as_ref)
                .ok_or(Error::Invalid)?,
        )
    }
    pub fn configuration(&self) -> Result<Configuration> {
        Ok(Configuration {
            ang_accel: self.a(1)?,
            ang_tc: self.a(2)?,
            thr_pit_ff: self.a(4)?,
            spd_p: self.spd_p,
            spd_i: self.spd_i,
            roll_corr_tc: self.a(8)?,
            time_corr_p: self.a(11)?,
            err_corr_p: self.a(12)?,
            err_corr_d: self.a(13)?,
            entry_rate: self.a(14)?,
            thr_lookahead: self.a(15)?,
            thr_min: self.a(17)?,
            thr_boost: self.a(18)?,
            yaw_accel: self.a(19)?,
            lookahead: self.a(20)?,
            path_scale: self.a(21)?,
            ke_rudd: self.a(26)?,
            ke_rudd_lookahead: self.a(27)?,
            alt_abort: self.a(28)?,
            airspeed_min: read(&self.airspeed_min)?,
            scaling_speed: read(&self.scaling_speed)?,
            airspeed_cruise: read(&self.airspeed_cruise)?,
        })
    }
}

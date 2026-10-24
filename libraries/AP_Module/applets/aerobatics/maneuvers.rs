// SPDX-License-Identifier: GPL-3.0-or-later
//! Maneuvers used by FixedWing/Schedules/AirShow.txt, including its two custom functions.

use super::{
    math::*,
    path::{Evaluation, Path},
    primitives::{Geometry as G, Roll as R},
};
use alloc::vec::Vec;
use ap_rust_sdk::{Context, Error, Result};

#[derive(Clone, Copy, Debug)]
pub enum Command {
    Loop,
    HalfReverseCuban,
    ScaleFigureEight,
    Immelmann,
    StraightRoll,
    SplitS,
    RollingCircle,
    HumptyBump,
    HalfCuban,
    BarrelRoll,
    CrossboxTophat,
    TriangularLoop,
    AlignBox,
    AlignCenter,
}

impl Command {
    // IDs and display names from Lua's command_table. Other built-ins are
    // not yet ported; loadable numeric schedules continue to use file IDs.
    pub fn builtin(id: i32) -> Option<(Self, &'static str)> {
        Some(match id {
            2 => (Self::Loop, "Loop"),
            7 => (Self::StraightRoll, "Axial Roll"),
            8 => (Self::RollingCircle, "Rolling Circle"),
            9 => (Self::HalfCuban, "Half Cuban Eight"),
            10 => (Self::HalfReverseCuban, "Half Reverse Cuban Eight"),
            12 => (Self::HumptyBump, "Humpty Bump"),
            14 => (Self::ScaleFigureEight, "Scale Figure Eight"),
            15 => (Self::Immelmann, "Immelmann Turn"),
            16 => (Self::SplitS, "Split-S"),
            26 => (Self::BarrelRoll, "Barrel Roll"),
            _ => return None,
        })
    }

    pub fn named(name: &str) -> Option<Self> {
        Some(match name {
            "loop" => Self::Loop,
            "half_reverse_cuban_eight" => Self::HalfReverseCuban,
            "scale_figure_eight" => Self::ScaleFigureEight,
            "immelmann_turn" => Self::Immelmann,
            "straight_roll" => Self::StraightRoll,
            "split_s" => Self::SplitS,
            "rolling_circle" => Self::RollingCircle,
            "humpty_bump" => Self::HumptyBump,
            "half_cuban_eight" => Self::HalfCuban,
            "barrel_roll" => Self::BarrelRoll,
            "crossbox_tophat" => Self::CrossboxTophat,
            "triangular_loop" => Self::TriangularLoop,
            "align_box" => Self::AlignBox,
            "align_center" => Self::AlignCenter,
            _ => return None,
        })
    }
}

struct Pieces<'a, 'b> {
    context: &'a Context<'b>,
    paths: Vec<Path>,
}

impl Pieces<'_, '_> {
    fn add(&mut self, geometry: G, roll: R) -> Result<&mut Path> {
        self.paths.try_reserve(1).map_err(|_| Error::NoMemory)?;
        self.paths.push(Path::leaf(self.context, geometry, roll)?);
        Ok(self.paths.last_mut().unwrap())
    }
    fn straight(&mut self, distance: f32, roll: f32) -> Result {
        self.add(G::Straight(distance), R::Angle(roll))?;
        Ok(())
    }
    fn vertical(&mut self, radius: f32, angle: f32) -> Result {
        self.add(G::Vertical { radius, angle }, R::Angle(0.0))?;
        Ok(())
    }
    fn horizontal(&mut self, radius: f32, angle: f32, bank: f32) -> Result {
        self.add(
            G::Horizontal {
                radius,
                angle,
                height: 0.0,
            },
            R::EntryExit(bank),
        )?;
        Ok(())
    }
}

pub fn build(
    c: &Context<'_>,
    command: Command,
    args: [f32; 4],
    box_width: f32,
    eval: Evaluation,
) -> Result<Path> {
    if args.iter().any(|x| !x.is_finite())
        || !eval.speed.is_finite()
        || eval.speed <= 0.0
        || !eval.entry_rate.is_finite()
        || eval.entry_rate <= 0.0
    {
        return Err(Error::Invalid);
    }
    let [r, arg2, arg3, _] = args;
    let a = r.abs();
    let mut p = Pieces {
        context: c,
        paths: Vec::new(),
    };
    match command {
        Command::AlignBox => return Ok(Path::align(Some(r))),
        Command::AlignCenter => return Ok(Path::align(None)),
        Command::Loop => {
            let loops = if arg3 <= 0.0 { 1.0 } else { arg3 };
            p.add(
                G::Vertical {
                    radius: r,
                    angle: 360.0 * loops,
                },
                R::EntryExit(arg2),
            )?;
        }
        Command::HalfReverseCuban => {
            p.vertical(r, 45.0)?;
            p.straight(2.0 * a / 3.0, 0.0)?;
            p.straight(2.0 * a / 3.0, 180.0)?;
            p.straight(2.0 * a / 3.0, 0.0)?;
            p.vertical(-r, 225.0)?;
        }
        Command::HalfCuban => {
            p.straight(2.0 * a * c.libm().sqrtf(2.0)?, 0.0)?;
            p.vertical(r, 225.0)?;
            p.straight(2.0 * a / 3.0, 0.0)?;
            p.straight(2.0 * a / 3.0, 180.0)?;
            p.straight(2.0 * a / 3.0, 0.0)?;
            p.vertical(-r, 45.0)?;
        }
        Command::ScaleFigureEight => {
            p.straight(a, 0.0)?;
            p.horizontal(r, 90.0, arg2)?;
            p.horizontal(-r, 360.0, -arg2)?;
            p.horizontal(r, 270.0, arg2)?;
            p.straight(3.0 * a, 0.0)?;
        }
        Command::Immelmann => {
            p.vertical(r, 180.0)?;
            p.straight(a, 180.0)?;
        }
        Command::StraightRoll => p.straight(r, arg2 * 360.0)?,
        Command::SplitS => {
            p.straight(a / 2.0, 180.0)?;
            p.vertical(-r, 180.0)?;
        }
        Command::RollingCircle => {
            p.add(
                G::Horizontal {
                    radius: r,
                    angle: 360.0,
                    height: 0.0,
                },
                R::Angle(360.0 * arg2),
            )?
            .attributes
            .thr_boost = Some(true);
        }
        Command::HumptyBump => {
            if arg2 < 2.0 * r {
                return Err(Error::Invalid);
            }
            p.vertical(r, 90.0)?;
            p.straight((arg2 - 2.0 * a) / 3.0, 0.0)?;
            p.straight((arg2 - 2.0 * a) / 3.0, 180.0)?;
            p.straight((arg2 - 2.0 * a) / 3.0, 0.0)?;
            p.vertical(-r, 180.0)?;
            p.straight(arg2 - 2.0 * a, 0.0)?;
            p.vertical(-r, 90.0)?;
            p.straight(2.0 * a, 0.0)?;
        }
        Command::BarrelRoll => {
            if r == 0.0 || arg3 <= 0.0 {
                return Err(Error::Invalid);
            }
            let gamma = deg(c.libm().atan2f((arg2 / arg3) / (2.0 * PI * a), 1.0)?);
            let bank = super::path::barrel_bank(c, r, eval.speed)?;
            let r2 = r / (1.0 - c.libm().cosf(rad(90.0 - gamma))?);
            p.horizontal(-r2, 90.0 - gamma, -bank)?;
            p.add(G::cylinder(c, r, arg2, arg3)?, R::Angle(0.0))?;
            p.horizontal(r2, 90.0 - gamma, bank)?;
        }
        Command::CrossboxTophat => {
            p.vertical(r, 90.0)?;
            p.add(
                G::Straight(arg2 - 2.0 * r),
                R::sequence(&[(1.0, 0.0), (1.0, 90.0), (1.0, 0.0)])?,
            )?
            .attributes
            .set_orient = Some(orient(c, 0.0, 90.0, 90.0)?);
            p.add(
                G::Vertical {
                    radius: -r,
                    angle: 90.0,
                },
                R::Angle(0.0),
            )?
            .attributes
            .set_orient = Some(orient(c, 180.0, 0.0, 90.0)?);
            p.straight(arg3 - 2.0 * r, 0.0)?;
            p.add(
                G::Vertical {
                    radius: r,
                    angle: 90.0,
                },
                R::Angle(0.0),
            )?
            .attributes
            .set_orient = Some(orient(c, 0.0, -90.0, 90.0)?);
            p.add(
                G::Straight(arg2 - 2.0 * r),
                R::sequence(&[(1.0, 0.0), (1.0, -90.0), (1.0, 0.0)])?,
            )?
            .attributes
            .set_orient = Some(orient(c, 0.0, -90.0, 180.0)?);
            p.add(
                G::Vertical {
                    radius: r,
                    angle: 90.0,
                },
                R::Angle(0.0),
            )?
            .attributes
            .set_orient = Some(orient(c, 0.0, 0.0, 180.0)?);
        }
        Command::TriangularLoop => {
            let h3 = arg2 - 2.0 * r;
            let side = h3 / c.libm().cosf(rad(45.0))?;
            let base = 2.0 * (h3 + r) - 2.0 * r;
            p.straight(base * 1.0 / 5.0, 180.0)?;
            p.straight(base * 2.0 / 5.0, 0.0)?;
            p.vertical(r, 135.0)?;
            for corner in [90.0, 135.0] {
                p.straight(side * 2.0 / 9.0, 0.0)?;
                p.straight(side * 2.0 / 9.0, 90.0)?;
                p.straight(side * 1.0 / 9.0, 0.0)?;
                p.straight(side * 2.0 / 9.0, 90.0)?;
                p.straight(side * 2.0 / 9.0, 0.0)?;
                p.vertical(r, corner)?;
            }
            p.straight(base * 2.0 / 5.0, 0.0)?;
            p.straight(base * 1.0 / 5.0, 180.0)?;
            p.straight(base * 2.0 / 5.0, 0.0)?;
        }
    }
    let mut path = Path::compose(c, p.paths, box_width, eval)?;
    if matches!(command, Command::BarrelRoll) {
        path.speed_dependent = Some(super::path::SpeedDependent::BarrelBank { radius: r });
    }
    Ok(path)
}

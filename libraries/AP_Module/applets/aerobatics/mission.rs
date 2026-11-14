// SPDX-License-Identifier: GPL-3.0-or-later
//! Mission entry heading, with bounded searches for malformed/jumping missions.

use super::{math, support::retry};
use ap_rust_sdk::{Context, Error, Location, MissionItem, Result};

pub const AUTO: u8 = 10;
pub const NAV_SCRIPT_TIME: u16 = 42702;

fn waypoint(
    mut index: u16,
    forward: bool,
    mut item: impl FnMut(u16) -> Result<Option<MissionItem>>,
    mut has_location: impl FnMut(u16) -> Result<bool>,
) -> Result<MissionItem> {
    // Bound both long runs of non-location commands and DO_JUMP cycles.
    for _ in 0..256 {
        let m = item(index)?.ok_or(Error::Unavailable)?;
        if has_location(m.command)? && (m.x != 0 || m.y != 0) {
            return Ok(m);
        }
        index = if forward && m.command == 177 {
            if !m.param1.is_finite() || m.param1 < 0.0 || m.param1 > u16::MAX as f32 {
                return Err(Error::Invalid);
            }
            m.param1 as u16
        } else if forward {
            index.checked_add(1).ok_or(Error::Unavailable)?
        } else {
            index.checked_sub(1).ok_or(Error::Unavailable)?
        };
    }
    Err(Error::Invalid)
}

fn location(c: &Context<'_>, index: u16, forward: bool) -> Result<Location> {
    let item = waypoint(
        index,
        forward,
        |i| retry(c, || c.mission().get_item(i)),
        |cmd| retry(c, || c.mission().cmd_has_location(cmd)),
    )?;
    let mut loc = Location::default();
    loc.set_lat(c, item.x)?;
    loc.set_lng(c, item.y)?;
    Ok(loc)
}

fn entry_heading(
    angle: f32,
    course: f32,
    bearing: impl FnOnce() -> Result<(u16, f32)>,
    mut wrap: impl FnMut(f32) -> Result<f32>,
) -> Result<f32> {
    if !angle.is_finite() || !course.is_finite() {
        return Err(Error::Invalid);
    }
    if angle != 0.0 {
        return Ok(angle);
    }
    let (index, mut yaw) = bearing()?;
    if index > 1 && wrap(course - yaw)?.abs() > 90.0 {
        yaw = wrap(yaw + 180.0)?;
    }
    Ok(yaw)
}

pub fn heading(c: &Context<'_>, angle: f32, course: f32) -> Result<f32> {
    entry_heading(
        angle,
        course,
        || {
            let index = retry(c, || c.mission().get_current_nav_index())?;
            let mut previous = if index > 1 {
                location(c, index - 1, false)?
            } else {
                retry(c, || c.ahrs().get_location())?.ok_or(Error::Unavailable)?
            };
            let mut next = location(c, index.checked_add(1).ok_or(Error::Invalid)?, true)?;
            Ok((
                index,
                math::deg(retry(c, || previous.get_bearing(c, &mut next))?),
            ))
        },
        |v| math::wrap_180(c, v),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entry_heading_matches_lua_turnaround_rules() {
        let wrap = |v: f32| {
            let v = v.rem_euclid(360.0);
            Ok(if v > 180.0 { v - 360.0 } else { v })
        };
        assert_eq!(
            entry_heading(123.0, 0.0, || panic!("fixed heading queried mission"), wrap),
            Ok(123.0)
        );
        for (index, course, bearing, expected) in [
            (4, 175.0, 170.0, 170.0),
            (4, 175.0, -10.0, 170.0),
            (4, -175.0, 175.0, 175.0),
            (4, 90.0, 0.0, 0.0),
            (4, 91.0, 0.0, 180.0),
            (1, 175.0, -10.0, -10.0),
        ] {
            assert_eq!(
                entry_heading(0.0, course, || Ok((index, bearing)), wrap),
                Ok(expected)
            );
        }
        assert_eq!(
            entry_heading(f32::NAN, 0.0, || Ok((1, 0.0)), wrap),
            Err(Error::Invalid)
        );
    }

    fn item(command: u16, x: i32, param1: f32) -> MissionItem {
        let mut m = MissionItem::default();
        m.command = command;
        m.x = x;
        m.param1 = param1;
        m
    }

    #[test]
    fn skips_script_arguments_and_resolves_jumps() {
        let items = [
            item(16, 100, 0.0),
            item(NAV_SCRIPT_TIME, 13, 72.0),
            item(177, 0, 4.0),
            item(16, 200, 0.0),
            item(16, 300, 0.0),
        ];
        let get = |i: u16| Ok(items.get(i as usize).copied());
        let located = |cmd| Ok(cmd == 16);
        assert_eq!(waypoint(1, false, get, located).unwrap().x, 100);
        assert_eq!(waypoint(2, true, get, located).unwrap().x, 300);
    }

    #[test]
    fn missing_waypoint_and_jump_cycle_are_bounded() {
        let get = |_: u16| Ok(Some(item(177, 0, 0.0)));
        assert!(matches!(
            waypoint(0, true, get, |_| Ok(false)),
            Err(Error::Invalid)
        ));
        assert!(matches!(
            waypoint(0, false, get, |_| Ok(false)),
            Err(Error::Unavailable)
        ));
        assert!(matches!(
            waypoint(1, true, |_| Ok(None), |_| Ok(false)),
            Err(Error::Unavailable)
        ));
    }
}

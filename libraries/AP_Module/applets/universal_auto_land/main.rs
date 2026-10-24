// SPDX-License-Identifier: GPL-3.0-or-later
//! Native port of AP_Scripting/applets/UniversalAutoLand.lua.
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Error, MissionItem, Parameter, Result};
use core::fmt::{self, Write};

ap_rust_sdk::module!(run);

const PARAM_TABLE_KEY: u8 = 80;
const PARAM_TABLE_PREFIX: &str = "ALAND_";

const FRAME_GLOBAL: u8 = 3;
const NAV_WAYPOINT: u16 = 16;
const NAV_LAND: u16 = 21;
const NAV_TAKEOFF: u16 = 22;
const DO_LAND_START: u16 = 189;
const TAKEOFF_PITCH: f32 = 15.0;

struct Text<const N: usize> {
    bytes: [u8; N],
    length: usize,
}

impl<const N: usize> Text<N> {
    fn new(args: fmt::Arguments<'_>) -> Result<Self> {
        let mut text = Self {
            bytes: [0; N],
            length: 0,
        };
        text.write_fmt(args).map_err(|_| Error::Invalid)?;
        Ok(text)
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.length]).unwrap()
    }
}

impl<const N: usize> Write for Text<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.length + value.len();
        if end > self.bytes.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.length..end].copy_from_slice(value.as_bytes());
        self.length = end;
        Ok(())
    }
}

fn retry<T>(context: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
    let start = context.time_us();
    loop {
        match call() {
            Err(Error::Busy) if context.time_us().saturating_sub(start) < 100_000 => {
                context.sleep_ms(1)?;
            }
            result => return result,
        }
    }
}

fn message(context: &Context<'_>, severity: i32, args: fmt::Arguments<'_>) -> Result {
    let text = Text::<128>::new(args)?;
    retry(context, || {
        context.gcs().send_text(MavSeverity(severity), text.as_str())
    })
}

fn bind_parameter<'a, 'h>(context: &'a Context<'h>, name: &str) -> Result<Parameter<'a, 'h>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Invalid)
}

fn read_parameter(parameter: &Parameter<'_, '_>) -> Result<f32> {
    let value = parameter.get()?.ok_or(Error::Invalid)?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(Error::Invalid)
    }
}

fn add_parameter<'a, 'h>(
    context: &'a Context<'h>,
    index: u8,
    name: &str,
    full_name: &str,
    default: f32,
) -> Result<Parameter<'a, 'h>> {
    if !retry(context, || {
        context.param().add_param(PARAM_TABLE_KEY, index, name, default)
    })? {
        return Err(Error::Host);
    }
    bind_parameter(context, full_name)
}

fn store_item(context: &Context<'_>, index: u16, mut item: MissionItem) -> Result {
    if retry(context, || context.mission().set_item(index, &mut item))? {
        Ok(())
    } else {
        Err(Error::Host)
    }
}

fn home_item(
    context: &Context<'_>,
    sequence: u16,
    command: u16,
    altitude: f32,
) -> Result<MissionItem> {
    let home = retry(context, || context.ahrs().get_home())?;
    let mut item = MissionItem::default();
    item.seq = sequence;
    item.frame = FRAME_GLOBAL;
    item.command = command;
    item.param1 = if command == NAV_TAKEOFF {
        TAKEOFF_PITCH
    } else if command == NAV_LAND {
        15.0
    } else {
        0.0
    };
    item.x = home.lat();
    item.y = home.lng();
    item.z = altitude;
    Ok(item)
}

fn setup_mission(context: &Context<'_>) -> Result {
    if !retry(context, || context.mission().clear())? {
        return Err(Error::Host);
    }

    let mut first = MissionItem::default();
    first.command = NAV_WAYPOINT;
    store_item(context, 0, first)?;

    let takeoff_altitude = retry(context, || context.param().get("TKOFF_ALT"))?
        .filter(|value| value.is_finite())
        .ok_or(Error::Invalid)?;
    store_item(
        context,
        1,
        home_item(context, 1, NAV_TAKEOFF, takeoff_altitude)?,
    )?;
    let mut land_start = MissionItem::default();
    land_start.seq = 2;
    land_start.frame = FRAME_GLOBAL;
    land_start.command = DO_LAND_START;
    store_item(context, 2, land_start)
}

fn wrap_180(mut angle: f32) -> f32 {
    while angle > 180.0 {
        angle -= 360.0;
    }
    while angle < -180.0 {
        angle += 360.0;
    }
    angle
}

fn capture_landing_direction(
    context: &Context<'_>,
    bearing: f32,
    distance: f32,
    altitude: f32,
) -> Result {
    let mut location = retry(context, || context.ahrs().get_home())?;
    retry(context, || {
        location.offset_bearing(context, wrap_180(bearing + 180.0), distance)
    })?;
    let mut approach = MissionItem::default();
    approach.seq = 3;
    approach.frame = FRAME_GLOBAL;
    approach.command = NAV_WAYPOINT;
    approach.x = location.lat();
    approach.y = location.lng();
    approach.z = altitude;
    store_item(context, 3, approach)?;
    store_item(context, 4, home_item(context, 4, NAV_LAND, 0.0)?)
}

fn run(context: &Context<'_>) -> Result {
    if !retry(context, || {
        context.param().add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4)
    })? {
        return Err(Error::Host);
    }

    let enable = read_parameter(&add_parameter(context, 1, "ENABLE", "ALAND_ENABLE", 1.0)?)?;
    let final_altitude =
        read_parameter(&add_parameter(context, 2, "WP_ALT", "ALAND_WP_ALT", 0.0)?)?;
    let final_distance =
        read_parameter(&add_parameter(context, 3, "WP_DIST", "ALAND_WP_DIST", 0.0)?)?;

    message(context, 6, format_args!("Loaded UniversalAutoLand Rust"))?;
    if enable != 1.0 {
        message(
            context,
            2,
            format_args!("Script disabled by AUTOLAND_ENABLE"),
        )?;
        return Ok(());
    }
    if final_altitude == 0.0 || final_distance == 0.0 {
        message(
            context,
            2,
            format_args!("Must set Final Waypoint alt and dist values!"),
        )?;
        return Ok(());
    }

    setup_mission(context)?;
    message(
        context,
        2,
        format_args!("Set Final Waypoint alt and dist values!"),
    )?;
    let mut mission_loaded = false;

    loop {
        let update = (|| -> Result<u32> {
            if !retry(context, || context.arming().is_armed())? {
                mission_loaded = false;
                return Ok(1_000);
            }
            if !mission_loaded {
                let location_available = retry(context, || context.ahrs().get_location())?.is_some();
                let speed = retry(context, || context.gps().ground_speed(0))?;
                let airspeed_min = retry(context, || context.param().get("AIRSPEED_MIN"))?
                    .filter(|value| value.is_finite())
                    .ok_or(Error::Invalid)?;
                if location_available && speed > 0.5 * airspeed_min {
                    let bearing = retry(context, || context.gps().ground_course(0))?;
                    let altitude = retry(context, || context.baro().get_altitude())?;
                    capture_landing_direction(context, bearing, final_distance, final_altitude)?;
                    mission_loaded = true;
                    message(
                        context,
                        5,
                        format_args!(
                            "Captured initial takeoff direction = {:.1} at {:.1} m and {:.1} m/s",
                            bearing, altitude, speed
                        ),
                    )?;
                }
            }
            Ok(200)
        })();

        let delay = match update {
            Ok(delay) => delay,
            Err(Error::Busy) => 10,
            Err(error) => return Err(error),
        };
        context.sleep_ms(delay)?;
    }
}

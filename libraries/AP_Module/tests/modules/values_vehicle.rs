// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Location, MissionItem, Result};
use core::fmt::{self, Write};

ap_rust_sdk::module!(run);

struct Text {
    bytes: [u8; 96],
    len: usize,
}

impl Write for Text {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.len + value.len();
        let dest = self.bytes.get_mut(self.len..end).ok_or(fmt::Error)?;
        dest.copy_from_slice(value.as_bytes());
        self.len = end;
        Ok(())
    }
}

fn report(context: &Context<'_>, args: fmt::Arguments<'_>) -> Result {
    let mut text = Text {
        bytes: [0; 96],
        len: 0,
    };
    text.write_fmt(args).map_err(|_| Error::Host)?;
    context.text(
        6,
        core::str::from_utf8(&text.bytes[..text.len]).map_err(|_| Error::Host)?,
    )
}

fn retry<T>(context: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
    loop {
        match call() {
            Err(Error::Busy) => context.sleep_ms(1)?,
            value => return value,
        }
    }
}

fn run(context: &Context<'_>) -> Result {
    if retry(context, || context.arming().is_armed())? {
        return Err(Error::Host);
    }
    let mut item = MissionItem::default();
    item.command = 16;
    item.frame = 3;
    item.x = -353632610;
    item.y = 1491652300;
    item.z = 42.0;
    item.param2 = 3.0; // Plane stores waypoint acceptance radius, not param1 delay.
    if !retry(context, || context.mission().set_item(0, &mut item))?
        || !retry(context, || context.mission().set_item(1, &mut item))?
    {
        return Err(Error::Host);
    }
    loop {
        let item = retry(context, || context.mission().get_item(1))?.ok_or(Error::Host)?;
        report(
            context,
            format_args!(
                "VAL M {} {} {} {} {} {}",
                item.command, item.frame, item.x, item.y, item.z as i32, item.param2 as i32
            ),
        )?;
        let mut location = Location::default();
        location.set_lat(context, -353632610)?;
        location.set_lng(context, 1491652300)?;
        location.set_alt(context, 58400)?;
        let mut other = location.copy(context)?;
        other.offset(context, 50.0, -30.0)?;
        let distance = location.get_distance(context, &mut other)?;
        let bearing = location.get_bearing(context, &mut other)?;
        let ne = location.get_distance_ne(context, &mut other)?;
        for (name, value) in [
            ("VAL_D", distance),
            ("VAL_B", bearing),
            ("VAL_N", ne.x),
            ("VAL_E", ne.y),
        ] {
            retry(context, || context.gcs().send_named_float(name, value))?;
        }
        if let Some(current) = retry(context, || context.ahrs().get_location())? {
            let home = retry(context, || context.ahrs().get_home())?;
            report(
                context,
                format_args!("VAL L {} {}", current.lat(), current.lng()),
            )?;
            report(context, format_args!("VAL H {} {}", home.lat(), home.lng()))?;
        }
        context.sleep_ms(200)?;
    }
}

// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::LocationAltFrame, Context, Error, Location, Result, Vector2f};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    for _ in 0..100 {
        let mut location = Location::default();
        location.set_lat(context, -353632610)?;
        location.set_lng(context, 1491652300)?;
        location.set_alt(context, 58400)?;
        if location.set_lat(context, 900000001) != Err(Error::Invalid)
            || location.set_alt(context, 8300000) != Err(Error::Invalid)
            || location.lat() != -353632610
            || location.alt() != 58400
        {
            return Err(Error::Host);
        }
        let mut other = location.copy(context)?;
        other.offset(context, 50.0, -30.0)?;
        let distance = location.get_distance(context, &mut other)?;
        let bearing = location.get_bearing(context, &mut other)?;
        let ned = location.get_distance_ned(context, &mut other)?;
        let ne = location.get_distance_ne(context, &mut other)?;
        if (distance - 58.3).abs() > 0.1
            || (ned.x - 50.0).abs() > 0.1
            || (ned.y + 30.0).abs() > 0.1
            || ne.x != ned.x
            || ne.y != ned.y
            || !(5.7..5.8).contains(&bearing)
            || location.get_alt_m(context, LocationAltFrame(0))? != Some(584.0)
        {
            return Err(Error::Host);
        }
        let mut v = Vector2f { x: 3.0, y: 4.0 };
        if v.length(context)? != 5.0 || v.add(context, &mut v.clone())?.x != 6.0 {
            return Err(Error::Host);
        }
        let mut other = Vector2f { x: 1.0, y: 7.0 };
        if v.sub(context, &mut other)? != (Vector2f { x: 2.0, y: -3.0 })
            || v != (Vector2f { x: 3.0, y: 4.0 })
            || other != (Vector2f { x: 1.0, y: 7.0 })
        {
            return Err(Error::Host);
        }
    }
    context.text(6, "MATH ok")
}

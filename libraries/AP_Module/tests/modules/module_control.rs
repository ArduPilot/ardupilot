// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::enums::{ApArmingMethod, ModeReason};
use ap_rust_sdk::{Context, Error, NotifyFlags, Result};

ap_rust_sdk::module!(run);

const LAND: u8 = 9;

fn require(value: bool, what: &str, context: &Context<'_>) -> Result {
    if value {
        return Ok(());
    }
    context.text(3, what)?;
    Err(Error::Host)
}

fn wait_for(context: &Context<'_>, mut done: impl FnMut() -> Result<bool>) -> Result {
    while !done()? {
        context.sleep_ms(50)?;
    }
    Ok(())
}

fn run(context: &Context<'_>) -> Result {
    let param = context.param();
    require(param.add_table(195, "MCT_", 3)?, "MCT fail table", context)?;
    require(param.add_param_int32(195, 1, "I32", 16777216)?, "MCT fail int32", context)?;
    require(param.add_param(195, 2, "F", 1.5)?, "MCT fail float", context)?;
    require(!param.add_param_int32(195, 2, "F", 0)?, "MCT fail type change", context)?;
    require(!param.add_param_int32(195, 3, "INEXACT", 16777217)?, "MCT fail inexact default", context)?;
    // exact int32 access through bound handles, without name lookups
    let bind = |name| context.parameter(name)?.ok_or(Error::Host);
    let (i32p, f, fence) = (bind("MCT_I32")?, bind("MCT_F")?, bind("FENCE_ENABLE")?);
    require(f.get_int32()?.is_none(), "MCT fail float as int", context)?;
    require(i32p.set_int32(123456789)?, "MCT fail set", context)?;
    // a float round trip would give 123456792
    require(i32p.get_int32()? == Some(123456789), "MCT fail exact", context)?;
    require(!f.set_int32(1)?, "MCT fail set float", context)?;
    require(!fence.set_int32(300)?, "MCT fail int8 range", context)?;
    require(i32p.set_and_save_int32(-7)?, "MCT fail save", context)?;
    require(i32p.get_int32()? == Some(-7), "MCT fail saved value", context)?;
    // the autotest's defaults file sets MCD_I32 16777216, MCD_BIG 2147483648
    // (out of range, ignored), MCD_RO 3 @READONLY and MCD_MAX 2147483647
    // (not exact as a float, ignored)
    require(param.add_table(196, "MCD_", 4)?, "MCT fail defaults table", context)?;
    require(param.add_param_int32(196, 1, "I32", 0)?, "MCT fail defaults int32", context)?;
    require(param.add_param_int32(196, 2, "BIG", 5)?, "MCT fail defaults big", context)?;
    require(param.add_param(196, 3, "RO", 0.0)?, "MCT fail defaults ro", context)?;
    require(param.add_param_int32(196, 4, "MAX", 0)?, "MCT fail defaults max", context)?;
    require(bind("MCD_BIG")?.get_int32()? == Some(5), "MCT fail default range", context)?;
    require(bind("MCD_MAX")?.get_int32()? == Some(0), "MCT fail default max", context)?;
    // first boot: exact default, then save a value that rounds to the same
    // float; after a reboot the saved value must have been kept
    let mcd = bind("MCD_I32")?;
    match mcd.get_int32()? {
        Some(16777216) => {
            require(mcd.set_and_save_int32(16777217)?, "MCT fail save near default", context)?;
        }
        Some(16777217) => context.text(6, "MCT saved int32 kept")?,
        _ => require(false, "MCT fail exact default", context)?,
    }

    let fs = context.fs();
    let created = fs.mkdir("modules/mct_dir")?;
    let stat = fs.stat("modules/mct_dir")?;
    require(created && stat.map(|s| s.is_directory) == Some(true), "MCT fail mkdir", context)?;
    require(!fs.mkdir("modules/mct_dir")?, "MCT fail mkdir existing", context)?;

    let home = context.ahrs().get_home()?;
    let mut points = [home; 4];
    for (point, (north, east)) in points.iter_mut().zip([(200.0, 200.0), (200.0, -200.0),
                                                       (-200.0, -200.0), (-200.0, 200.0)]) {
        point.offset(context, north, east)?;
    }
    require(context.fence().write_inclusion_polygon(&points)?, "MCT fail fence", context)?;
    require(context.fence().get_breach_count()? == 0, "MCT fail breach count", context)?;
    require(context.notify().get_flags()? & NotifyFlags::ARMED == 0, "MCT fail flags disarmed", context)?;
    context.text(6, "MCT setup ok")?;

    let arming = context.arming();
    wait_for(context, || arming.is_armed())?;
    wait_for(context, || Ok(context.notify().get_flags()? & NotifyFlags::ARMED != 0))?;
    // let the test observe the armed state first
    context.sleep_ms(2000)?;
    require(arming.disarm_method(ApArmingMethod(5), false)? == false, "MCT fail method allowlist", context)?;
    require(arming.is_armed()?, "MCT fail still armed", context)?;
    require(arming.disarm_method(ApArmingMethod::FENCEBREACH, false)?, "MCT fail disarm", context)?;
    require(arming.last_disarm_method()? == ApArmingMethod::FENCEBREACH, "MCT fail last method", context)?;
    context.text(6, "MCT disarmed")?;

    let vehicle = context.vehicle();
    require(vehicle.set_mode_reason(LAND, ModeReason(1)) == Err(Error::Invalid), "MCT fail reason range", context)?;
    require(!vehicle.set_mode_reason(LAND, ModeReason(11))?, "MCT fail reason allowlist", context)?;
    require(vehicle.set_mode_reason(LAND, ModeReason::FENCE_BREACHED)?, "MCT fail mode", context)?;
    require(vehicle.get_mode()? == LAND && vehicle.get_control_mode_reason()? == 10, "MCT fail mode reason", context)?;
    context.text(6, "MCT mode ok")
}

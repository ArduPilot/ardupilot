// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

fn require(value: bool) -> Result {
    if value {
        Ok(())
    } else {
        Err(Error::Host)
    }
}

fn run(context: &Context<'_>) -> Result {
    if option_env!("AP_MODULE_PARAM_MODE") == Some("capacity") {
        return capacity(context);
    }
    require(context.param().get("") == Err(Error::Invalid))?;
    require(context.param().get("RTP_\0SAVED") == Err(Error::Invalid))?;
    require(context.param().get("12345678901234567") == Err(Error::Invalid))?;
    require(context.param().get("NO_SUCH_PARAM")? == None)?;
    require(!context.param().set("NO_SUCH_PARAM", 1.0)?)?;
    require(context.param().add_table(194, "RTP_", 3)?)?;
    require(context.param().add_param(194, 1, "SAVED", 10.0)?)?;
    require(context.param().add_param(194, 3, "CMD", 0.0)?)?;
    // Fill a gap only after the later index has been published.
    require(context.param().add_param(194, 2, "UNSAVED", 20.0)?)?;
    require(!context.param().add_param(194, 2, "RENAMED", 20.0)?)?;
    require(!context.param().add_table(194, "OTHER_", 3)?)?;
    require(!context.param().add_table(194, "RTP_", 4)?)?;
    require(context.param().set("RTP_SAVED", f32::NAN) == Err(Error::Invalid))?;
    require(context.param().set_default("RTP_UNSAVED", f32::INFINITY) == Err(Error::Invalid))?;
    require(context.param().add_param(194, 2, "UNSAVED", f32::NAN) == Err(Error::Invalid))?;
    let saved = context.param().get("RTP_SAVED")?.ok_or(Error::Host)?;
    require(saved == 10.0 || saved == 42.0)?;
    require(context.param().get("RTP_UNSAVED")? == Some(20.0))?;
    require(context.param().set("RTP_CMD", 0.0)?)?;
    require(context.parameter("NO_SUCH_PARAM")?.is_none())?;
    require(matches!(context.parameter(""), Err(Error::Invalid)))?;
    require(matches!(
        context.parameter("RTP_\0SAVED"),
        Err(Error::Invalid)
    ))?;
    require(matches!(
        context.parameter("12345678901234567"),
        Err(Error::Invalid)
    ))?;
    // Exercise allocation/drop repeatedly before retaining the loop's helpers.
    for _ in 0..256 {
        let mut parameter = context.parameter("RTP_UNSAVED")?.ok_or(Error::Host)?;
        require(parameter.get()? == Some(20.0))?;
        require(parameter.set(22.0)?)?;
        require(context.param().get("RTP_UNSAVED")? == Some(22.0))?;
        require(context.param().set("RTP_UNSAVED", 20.0)?)?;
        require(parameter.get()? == Some(20.0))?;
        require(!parameter.init("NO_SUCH_PARAM")?)?;
        require(parameter.get()?.is_none())?;
        require(!parameter.set(1.0)?)?;
        require(!parameter.configured()?)?;
        require(parameter.init("RTP_UNSAVED")?)?;
        require(parameter.set(f32::NAN) == Err(Error::Invalid))?;
        require(parameter.set_and_save(f32::INFINITY) == Err(Error::Invalid))?;
        require(parameter.set_default(f32::NEG_INFINITY) == Err(Error::Invalid))?;
    }
    let saved_parameter = context.parameter("RTP_SAVED")?.ok_or(Error::Host)?;
    let unsaved_parameter = context.parameter("RTP_UNSAVED")?.ok_or(Error::Host)?;
    let command = context.parameter("RTP_CMD")?.ok_or(Error::Host)?;
    if saved == 42.0 {
        require(saved_parameter.configured()?)?;
    }
    if option_env!("AP_MODULE_PARAM_MODE") != Some("renode") {
        for name in [
            "ACRO_ROLL_RATE",
            "ACRO_YAW_RATE",
            "AIRSPEED_MIN",
            "SCALING_SPEED",
            "MAV_SYSID",
            "AIRSPEED_CRUISE",
            "AHRS_TRIM_X",
        ] {
            let parameter = context.parameter(name)?.ok_or(Error::Host)?;
            require(parameter.get()? == context.param().get(name)?)?;
        }
        for (name, outside) in [
            ("MOD_THD_PRIO_MAX", 65536.0),
            ("MOD_RESTART", 32768.0),
            ("MOD_HEAP_MAX", 2147483648.0),
        ] {
            let before = context.param().get(name)?.ok_or(Error::Host)?;
            require(context.param().set(name, outside) == Err(Error::Invalid))?;
            require(context.param().set_and_save(name, outside) == Err(Error::Invalid))?;
            require(context.param().set_default(name, outside) == Err(Error::Invalid))?;
            require(context.param().get(name)? == Some(before))?;
            let parameter = context.parameter(name)?.ok_or(Error::Host)?;
            require(parameter.get()? == Some(before))?;
            require(parameter.set(outside) == Err(Error::Invalid))?;
            require(parameter.set_and_save(outside) == Err(Error::Invalid))?;
            require(parameter.set_default(outside) == Err(Error::Invalid))?;
            require(parameter.get()? == Some(before))?;
        }
    }

    if option_env!("AP_MODULE_PARAM_MODE") == Some("renode") {
        require(context.ahrs().get_roll_rad() == Err(Error::Unavailable))?;
        require(context.arming().is_armed() == Err(Error::Unavailable))?;
        require(context.vehicle().get_mode() == Err(Error::Unavailable))?;
        require(context.gps().num_sensors() == Err(Error::Unavailable))?;
        require(context.rc().get_pwm(1) == Err(Error::Unavailable))?;
        if saved == 10.0 {
            require(saved_parameter.set_and_save(42.0)?)?;
            // The save API queues work, not a value snapshot. Allow the real
            // ChibiOS IO worker to finish before making an unsaved change.
            context.sleep_ms(100)?;
        } else {
            context.text(6, "PARAM restored")?;
        }
        require(saved_parameter.set(99.0)?)?;
        require(unsaved_parameter.set(99.0)?)?;
    }
    context.text(6, "PARAM ready")?;
    loop {
        if command.get()? == Some(1.0) {
            require(saved_parameter.set_and_save(42.0)?)?;
            require(unsaved_parameter.set(99.0)?)?;
            require(command.set(0.0)?)?;
            context.text(6, "PARAM queued")?;
        }
        if command.get()? == Some(2.0) {
            // Alter saved RAM only after the test has allowed IO to persist 42.
            require(saved_parameter.set(99.0)?)?;
            require(unsaved_parameter.set_default(21.0)?)?;
            require(command.set(0.0)?)?;
            context.text(6, "PARAM volatile")?;
        }
        if command.get()? == Some(3.0) {
            // Legacy Lua-created tables are accessible through the same helper.
            let lua = context.parameter("LUP_VALUE")?.ok_or(Error::Host)?;
            require(lua.get()? == Some(73.0))?;
            require(lua.set(76.0)?)?;
            require(context.param().get("LUP_VALUE")? == Some(76.0))?;
            require(context.param().set("LUP_VALUE", 73.0)?)?;
            require(lua.get()? == Some(73.0))?;
            require(command.set(0.0)?)?;
            context.text(6, "PARAM Lua helper")?;
        }
        match context.sleep_ms(20) {
            Ok(()) => {}
            Err(Error::Stopped) => {
                require(saved_parameter.get() == Err(Error::Stopped))?;
                require(matches!(
                    context.parameter("RTP_SAVED"),
                    Err(Error::Stopped)
                ))?;
                // Drop still releases all three helpers after stop.
                return Ok(());
            }
            Err(error) => return Err(error),
        }
    }
}

fn capacity(context: &Context<'_>) -> Result {
    let prefixes = [
        "R0_", "R1_", "R2_", "R3_", "R4_", "R5_", "R6_", "R7_", "R8_", "R9_",
    ];
    for (index, prefix) in prefixes.iter().enumerate() {
        require(context.param().add_table(
            180 + index as u8,
            prefix,
            if index == 0 { 63 } else { 1 },
        )?)?;
    }
    require(!context.param().add_table(190, "FULL_", 1)?)?;
    require(context.param().add_param(180, 63, "P63", 63.0)?)?;
    for index in 1..63 {
        let bytes = [b'P', b'0' + index / 10, b'0' + index % 10];
        let name = core::str::from_utf8(&bytes).map_err(|_| Error::Host)?;
        require(context.param().add_param(180, index, name, index as f32)?)?;
    }
    require(!context.param().add_param(180, 63, "RENAMED", 0.0)?)?;
    require(context.param().add_param(180, 64, "OUTSIDE", 0.0) == Err(Error::Invalid))?;
    require(context.param().get("R0_P01")? == Some(1.0))?;
    require(context.param().get("R0_P63")? == Some(63.0))?;
    context.text(6, "PARAM capacity")?;
    loop {
        context.sleep_ms(1000)?;
    }
}

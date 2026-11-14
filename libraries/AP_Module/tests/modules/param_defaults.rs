// SPDX-License-Identifier: GPL-3.0-or-later
// Exact int32 parameters and defaults-file values for parameters registered
// after boot, driven by the NativeModuleParamDefaults autotest.
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

fn require(value: bool, what: &str, context: &Context<'_>) -> Result {
    if value {
        return Ok(());
    }
    context.text(3, what)?;
    Err(Error::Host)
}

fn run(context: &Context<'_>) -> Result {
    let param = context.param();
    let bind = |name| context.parameter(name)?.ok_or(Error::Host);
    require(param.add_table(195, "PDT_", 3)?, "PDT fail table", context)?;
    require(
        param.add_param_int32(195, 1, "I32", 16777216)?,
        "PDT fail int32",
        context,
    )?;
    require(
        param.add_param(195, 2, "F", 1.5)?,
        "PDT fail float",
        context,
    )?;
    require(
        !param.add_param_int32(195, 2, "F", 0)?,
        "PDT fail type change",
        context,
    )?;
    require(
        !param.add_param_int32(195, 3, "INEXACT", 16777217)?,
        "PDT fail inexact default",
        context,
    )?;
    let (i32p, f) = (bind("PDT_I32")?, bind("PDT_F")?);
    require(f.get_int32()?.is_none(), "PDT fail float as int", context)?;
    require(i32p.set_int32(123456789)?, "PDT fail set", context)?;
    // a float round trip would give 123456792
    require(
        i32p.get_int32()? == Some(123456789),
        "PDT fail exact",
        context,
    )?;
    require(!f.set_int32(1)?, "PDT fail set float", context)?;
    require(
        !bind("FENCE_ENABLE")?.set_int32(300)?,
        "PDT fail int8 range",
        context,
    )?;

    // Defaults are float-backed; ignore out-of-range or inexact integer
    // overrides while retaining the complete int32 range for runtime values.
    require(
        param.add_table(196, "PDD_", 7)?,
        "PDT fail defaults table",
        context,
    )?;
    require(
        param.add_param_int32(196, 1, "I32", 0)?,
        "PDT fail defaults int32",
        context,
    )?;
    require(
        param.add_param_int32(196, 2, "BIG", 5)?,
        "PDT fail defaults big",
        context,
    )?;
    require(
        param.add_param(196, 3, "RO", 0.0)?,
        "PDT fail defaults ro",
        context,
    )?;
    require(
        param.add_param_int32(196, 4, "MAX", 9)?,
        "PDT fail defaults max",
        context,
    )?;
    require(
        param.add_param_int32(196, 5, "MIN", 0)?,
        "PDT fail defaults min",
        context,
    )?;
    require(
        param.add_param_int32(196, 6, "ROUND", 7)?,
        "PDT fail defaults round",
        context,
    )?;
    require(
        param.add_param_int32(196, 7, "LARGE", 0)?,
        "PDT fail defaults large",
        context,
    )?;
    require(
        bind("PDD_BIG")?.get_int32()? == Some(5),
        "PDT fail default range",
        context,
    )?;
    require(
        bind("PDD_MAX")?.get_int32()? == Some(9),
        "PDT fail inexact max ignored",
        context,
    )?;
    require(
        bind("PDD_MIN")?.get_int32()? == Some(i32::MIN),
        "PDT fail default min",
        context,
    )?;
    require(
        bind("PDD_ROUND")?.get_int32()? == Some(7),
        "PDT fail inexact default ignored",
        context,
    )?;
    require(
        bind("PDD_LARGE")?.get_int32()? == Some(1_000_000_000),
        "PDT fail large exact default",
        context,
    )?;
    // First boot: exact default, then save a value that rounds to the same
    // float. After a reboot the saved value must have been kept.
    let pdd = bind("PDD_I32")?;
    match pdd.get_int32()? {
        Some(16777216) => {
            require(
                pdd.set_and_save_int32(16777217)?,
                "PDT fail save near default",
                context,
            )?;
        }
        Some(16777217) => context.text(6, "PDT saved int32 kept")?,
        _ => require(false, "PDT fail exact default", context)?,
    }
    context.text(6, "PDT ready")?;
    loop {
        context.sleep_ms(1000)?;
    }
}

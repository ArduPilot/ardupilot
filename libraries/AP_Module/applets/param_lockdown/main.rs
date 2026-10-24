// SPDX-License-Identifier: GPL-3.0-or-later
//! Native port of AP_Scripting/applets/param-lockdown.lua.
#![no_std]
#![no_main]

mod mavlink;

use ap_rust_sdk::{enums::MavSeverity, Context, Error, Parameter, Result};
use core::fmt::{self, Write};
use mavlink::{decode_param_set, param_error, PARAM_ERROR_ID, PARAM_SET_ID};

ap_rust_sdk::module!(run);

const PARAM_TABLE_KEY: u8 = 92;
const PERMISSION_DENIED: u8 = 3;
const VALUE_OUT_OF_RANGE: u8 = 2;
const DOES_NOT_EXIST: u8 = 1;

struct Text<const N: usize> {
    bytes: [u8; N],
    length: usize,
}

impl<const N: usize> Text<N> {
    fn new(args: fmt::Arguments<'_>) -> Result<Self> {
        let mut text = Self { bytes: [0; N], length: 0 };
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
        if end > N {
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

fn text(context: &Context<'_>, severity: i32, args: fmt::Arguments<'_>) -> Result {
    let message = Text::<128>::new(args)?;
    retry(context, || context.gcs().send_text(MavSeverity(severity), message.as_str()))
}

fn allowed(name: &str) -> bool {
    matches!(
        name,
        "MAV_OPTIONS"
            | "PARAM_LOCK_ENAB"
            | "BATT_ARM_MAH"
            | "BATT_ARM_VOLT"
            | "BATT_CAPACITY"
            | "BATT_CRT_MAH"
            | "BATT_CRT_VOLT"
            | "BATT_FS_CRT_ACT"
            | "BATT_FS_LOW_ACT"
            | "BATT_LOW_MAH"
            | "BATT_LOW_VOLT"
            | "BRD_OPTIONS"
            | "COMPASS_USE3"
            | "FENCE_ACTION"
            | "FENCE_ALT_MAX"
            | "FENCE_ENABLE"
            | "FENCE_RADIUS"
            | "FENCE_TYPE"
            | "LIGHTS_ON"
            | "LOG_BITMASK"
            | "LOG_DISARMED"
            | "LOG_FILE_DSRMROT"
            | "RTL_ALT_M"
            | "RTL_LOIT_TIME"
            | "RTL_SPEED_MS"
    )
}

fn enabled(parameter: &Parameter<'_, '_>) -> Result<bool> {
    Ok(parameter.get()?.filter(|value| value.is_finite()).ok_or(Error::Invalid)? > 0.0)
}

fn handle_messages(context: &Context<'_>, active: bool) -> Result {
    while let Some(message) = context.mavlink().receive_chan()? {
        if !active {
            continue;
        }
        let Some(set) = decode_param_set(message.bytes()) else {
            continue;
        };
        let error = if allowed(set.name) {
            match retry(context, || context.param().set_and_save(set.name, set.value)) {
                Ok(true) => {
                    text(context, 4, format_args!("param-lockdown: param set applied"))?;
                    continue;
                }
                Ok(false) => DOES_NOT_EXIST,
                Err(Error::Invalid) => VALUE_OUT_OF_RANGE,
                Err(error) => return Err(error),
            }
        } else {
            PERMISSION_DENIED
        };
        text(
            context,
            4,
            format_args!("param-lockdown: param set denied ({})", set.name),
        )?;
        let payload = param_error(set.source_component, set.name, error);
        let _ = context.mavlink().send_chan(
            message.channel,
            PARAM_ERROR_ID,
            &payload,
            Some(set.source_system),
        )?;
    }
    Ok(())
}

fn run(context: &Context<'_>) -> Result {
    if !retry(context, || context.param().add_table(PARAM_TABLE_KEY, "PARAM_LOCK_", 7))?
        || !retry(context, || {
            context.param().add_param(PARAM_TABLE_KEY, 1, "ENAB", 1.0)
        })?
    {
        return Err(Error::Host);
    }
    let enable = retry(context, || context.parameter("PARAM_LOCK_ENAB"))?.ok_or(Error::Invalid)?;
    context.mavlink().init(5, 1)?;
    if !context.mavlink().register_rx_msgid(PARAM_SET_ID)? {
        return Err(Error::Host);
    }
    text(context, 6, format_args!("param-lockdown script loaded"))?;
    let mut allowing = retry(context, || context.gcs().get_allow_param_set())?;

    loop {
        let active = enabled(&enable)?;
        if active && allowing {
            retry(context, || context.gcs().set_allow_param_set(false))?;
            allowing = false;
        } else if !active && !allowing {
            retry(context, || context.gcs().set_allow_param_set(true))?;
            allowing = true;
        }
        handle_messages(context, active)?;
        context.sleep_ms(10)?;
    }
}

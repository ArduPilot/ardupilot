// SPDX-License-Identifier: GPL-3.0-or-later
use ap_rust_sdk::{Context, Error, OldParameterType, Result};

// Reserved for this applet. AP_Param rejects a persisted prefix/key collision.
const TABLE: u8 = 199;
const LEGACY_DYNAMIC_KEY: u16 = 500;
const LEGACY_RC_KEY: u16 = 64;
const FIELDS: [(&str, &str, f32, OldParameterType, u32); 5] = [
    (
        "UID1",
        "ELRS_UID1",
        0.0,
        OldParameterType::Int32,
        0xffffff,
    ),
    (
        "UID2",
        "ELRS_UID2",
        0.0,
        OldParameterType::Int32,
        0xffffff,
    ),
    (
        "MODEL",
        "ELRS_MODEL",
        255.0,
        OldParameterType::Int16,
        255,
    ),
    ("BIND", "ELRS_BIND", 0.0, OldParameterType::Int8, 1),
    ("VER", "ELRS_VER", 4.0, OldParameterType::Int8, 255),
];

fn checked(value: f32, max: u32) -> Result<u32> {
    if !value.is_finite() || value < 0.0 || value > max as f32 || value as u32 as f32 != value {
        return Err(Error::Invalid);
    }
    Ok(value as u32)
}

pub fn configure(context: &Context<'_>) -> Result<bool> {
    if !context.param().add_table(TABLE, "ELRS_", 5)? {
        return Err(Error::Invalid);
    }
    let mut values = [0u32; 5];
    for (i, (suffix, name, default, kind, max)) in FIELDS.iter().enumerate() {
        if !context
            .param()
            .add_param_typed(TABLE, (i + 1) as u8, suffix, *default, *kind as u8, i < 2)?
        {
            return Err(Error::Invalid);
        }
        let parameter = context.parameter(name)?.ok_or(Error::Invalid)?;
        if !parameter.configured()? {
            // Former nested RC group 11 takes precedence over the original flat layout.
            let nested =
                context
                    .param()
                    .old_value(LEGACY_RC_KEY, 11 + (((i + 1) as u32) << 6), *kind)?;
            // The old prefix owns key 500; its typed records precede its earlier Float records.
            let typed = context.param().old_value(LEGACY_DYNAMIC_KEY, (i + 1) as u32, *kind)?;
            let dynamic = match typed {
                Some(value) => Some(value),
                None => context.param().old_value(
                    LEGACY_DYNAMIC_KEY, (i + 1) as u32, OldParameterType::Float)?,
            };
            let old = match dynamic.or(nested) {
                Some(value) => Some(value),
                None => context
                    .param()
                    .old_value(LEGACY_RC_KEY, 6 + i as u32, *kind)?,
            };
            if let Some(value) = old {
                checked(value, *max)?;
                if !parameter.set_and_save(value)? {
                    return Err(Error::Invalid);
                }
            }
        }
        values[i] = checked(parameter.get()?.ok_or(Error::Invalid)?, *max)?;
    }
    let config = values[2] | (values[3] << 16) | (values[4] << 24);
    if !context
        .periph()
        .sx1280_configure(values[0], values[1], config)?
    {
        return Err(Error::Invalid);
    }
    Ok(values[3] == 1)
}

fn save(context: &Context<'_>, name: &str, value: u32) -> Result {
    if !context.param().set_and_save(name, value as f32)? {
        return Err(Error::Invalid);
    }
    Ok(())
}

pub fn update(context: &Context<'_>, pending_bind: &mut bool) -> Result {
    let report = context.periph().sx1280_snapshot(0)?;
    let flags = report.word0 >> 24;
    let error = (report.word0 >> 16) & 255;
    // Consume only the request used to initialize this receiver instance.
    // A later parameter write belongs to the next boot.
    if *pending_bind && flags & 64 != 0 {
        if context.param().get("ELRS_BIND")? == Some(1.0) {
            save(context, "ELRS_BIND", 0)?;
        }
        *pending_bind = false;
    }
    if flags & 2 != 0 && error == 0 {
        // Learned UID is immutable until reboot. Snapshot keeps its six bytes coherent.
        let uid = context.periph().sx1280_snapshot(104)?;
        let a = uid.word0.to_le_bytes();
        let b = uid.word1.to_le_bytes();
        save(
            context,
            "ELRS_UID1",
            (u32::from(a[2]) << 16) | (u32::from(a[3]) << 8) | u32::from(b[0]),
        )?;
        save(
            context,
            "ELRS_UID2",
            (u32::from(b[1]) << 16) | (u32::from(b[2]) << 8) | u32::from(b[3]),
        )?;
        context.periph().reboot(false)?;
    }
    Ok(())
}

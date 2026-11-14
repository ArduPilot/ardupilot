// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

ap_rust_sdk::param_table! {
    struct Params, enum P;
    key = 185, prefix = "RTBL_";
    Saved = 1, "SAVED", f32: 3.0;
    Last = 63, "LAST", f32: 63.0;
}

ap_rust_sdk::param_table! {
    struct Conditional, enum C;
    key = 186, prefix = "RTC_";
    size = 19;
    Enable = 1, "ENABLE", f32: 1.0;
    Hidden = 19, "HIDDEN", f32: 19.0;
}

fn run(context: &Context<'_>) -> Result {
    Conditional::register_table(context)?;
    Conditional::definition(C::Enable).register(context, Conditional::KEY)?;
    let mut cached = Params::register(context)?;
    let live = Params::bind(context, P::Saved)?;
    if cached.get(P::Last) != 63.0 {
        return Err(Error::Invalid);
    }
    context.text(6, "TABLE ready")?;
    loop {
        let value = live.get()?.ok_or(Error::Invalid)?;
        if value == 42.0 {
            if cached.get(P::Saved) != 3.0 || !P::Saved.changed(cached.refresh()?) {
                return Err(Error::Invalid);
            }
            cached.set_and_save(P::Saved, 43.0)?;
            if live.get()? != Some(43.0) {
                return Err(Error::Invalid);
            }
            Conditional::definition(C::Hidden).register(context, Conditional::KEY)?;
            context.text(6, "TABLE updated")?;
        }
        context.sleep_ms(20)?;
    }
}

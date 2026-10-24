// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    for _ in 0..100 {
        if context.libm().sinf(0.0)? != 0.0
            || context.libm().cosf(0.0)? != 1.0
            || context.libm().atan2f(0.0, 1.0)? != 0.0
            || context.libm().sqrtf(4.0)? != 2.0
            || context.libm().fmodf(5.5, 2.0)? != 1.5
            || context.libm().floorf(-1.5)? != -2.0
            || context.libm().ceilf(-1.5)? != -1.0
            || context.libm().asinf(0.0)? != 0.0
            || context.libm().acosf(1.0)? != 0.0
            || context.libm().expf(0.0)? != 1.0
            || context.libm().logf(1.0)? != 0.0
            || context.libm().powf(2.0, 3.0)? != 8.0
            || context.libm().tanf(0.0)? != 0.0
        {
            return Err(Error::Host);
        }
    }
    loop {
        for (name, value) in [
            ("LM_SIN", context.libm().sinf(0.7)?),
            ("LM_COS", context.libm().cosf(-0.7)?),
            ("LM_ATAN", context.libm().atan2f(0.7, -0.3)?),
            ("LM_SQRT", context.libm().sqrtf(2.0)?),
            ("LM_FMOD", context.libm().fmodf(-2.3, 0.7)?),
            ("LM_FLOOR", context.libm().floorf(-1.2)?),
            ("LM_CEIL", context.libm().ceilf(-1.2)?),
            ("LM_TAN", context.libm().tanf(0.3)?),
            ("LM_ASIN", context.libm().asinf(0.3)?),
            ("LM_ACOS", context.libm().acosf(-0.3)?),
            ("LM_EXP", context.libm().expf(0.7)?),
            ("LM_LOG", context.libm().logf(1.3)?),
            ("LM_POW", context.libm().powf(1.3, 2.7)?),
        ] {
            match context.gcs().send_named_float(name, value) {
                Err(Error::Unavailable) => return context.text(6, "MATH ok"),
                result => result?,
            }
        }
        context.sleep_ms(200)?;
    }
}

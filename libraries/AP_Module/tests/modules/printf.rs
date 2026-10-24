// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    let formatter = context.string();
    assert_eq!(
        formatter
            .format_printf("Tick %.1f", &[12.5f32.into()])?
            .as_str(),
        "Tick 12.5"
    );
    let args = [
        (-7i32).into(),
        12.5f64.into(),
        "abc".into(),
        (-1234567890123i64).into(),
        1234567890123456u64.into(),
        0xdeadbeefu32.into(),
        0.25f32.into(),
        65i32.into(),
    ];
    let mixed = formatter.format_printf("%d %.1f %s %lld %llu %08X %.2f %c %%", &args)?;
    assert!(
        mixed.as_str() == "-7 12.5 abc -1234567890123 1234567890123456 DEADBEEF 0.25 A %",
        "printf mixed"
    );
    let floats = [
        1.0f32.into(),
        2.0f32.into(),
        3.0f32.into(),
        4.0f32.into(),
        5.0f32.into(),
        6.0f32.into(),
        7.0f32.into(),
        8.0f32.into(),
    ];
    let numbers = formatter.format_printf("%.0f %.0f %.0f %.0f %.0f %.0f %.0f %.0f", &floats)?;
    assert!(numbers.as_str() == "1 2 3 4 5 6 7 8", "printf eight floats");
    assert_eq!(formatter.format_printf("%s", &["é".into()])?.as_str(), "é");
    assert_eq!(
        formatter
            .format_printf("%255s", &["".into()])?
            .as_str()
            .len(),
        255
    );
    for invalid in [
        "%d", "%s", "%n", "%p", "%*f", "%.*f", "%1$f", "%ld", "%llf", "%", "%f %f",
    ] {
        assert!(matches!(
            formatter.format_printf(invalid, &[12.5f32.into()]),
            Err(Error::Invalid)
        ));
    }
    assert!(matches!(
        formatter.format_printf("%255sX", &["".into()]),
        Err(Error::Invalid)
    ));
    assert!(matches!(
        formatter.format_printf("%.1s", &["é".into()]),
        Err(Error::Invalid)
    ));
    assert!(matches!(
        formatter.format_printf("%c", &[0i32.into()]),
        Err(Error::Invalid)
    ));
    context.text(6, "MATH ok")
}

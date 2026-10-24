// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result, SerialFlowControl, SerialParity, SerialStopBits};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    let driver = context.serial().find_serial(0)?.ok_or(Error::Unavailable)?;
    let device = context
        .serial().find_simulated_device(28, 0)?
        .ok_or(Error::Unavailable)?;

    driver.begin(context, Some(115200))?;
    driver.configure_parity(context, SerialParity::None)?;
    driver.set_stop_bits(context, SerialStopBits::One)?;
    driver.set_flow_control(context, SerialFlowControl::Disabled)?;
    driver.set_unbuffered_writes(context, false)?;

    let to_device = b"hello device";
    let mut received = [0u8; 12];
    if driver.write(context, to_device)? != to_device.len()
        || device.available(context)? < to_device.len() as u32
        || device.read(context, &mut received)? != received.len()
        || &received != to_device
    {
        return Err(Error::Host);
    }
    context.text(6, "Rust driver -> device good")?;

    let to_driver = b"hello driver";
    if device.write(context, to_driver)? != to_driver.len()
        || driver.available(context)? < to_driver.len() as u32
        || driver.read(context, &mut received)? != received.len()
        || &received != to_driver
    {
        return Err(Error::Host);
    }
    context.text(6, "Rust device -> driver good")
}

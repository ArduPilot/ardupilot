// SPDX-License-Identifier: GPL-3.0-or-later
//! Skybrush drone show support as a native module: show playback in a custom
//! flight mode, custom MAVLink protocol, fences and status telemetry.
#![no_std]
#![no_main]
extern crate alloc;

mod app;
mod coords;
mod crth;
mod fences;
mod geofence;
mod lights_backends;
mod lights_out;
mod loader;
mod logging;
mod mavlink;
mod monitors;
mod params;
#[allow(dead_code)]
mod show;
mod telemetry;
mod timing;
mod support;
mod vehicle;

use ap_rust_sdk::{enums::MavSeverity, Context, Error, Result};
use app::App;
use mavlink as mav;

ap_rust_sdk::module!(run);

const TICK_US: u64 = 20000;

fn setup_mavlink(c: &Context<'_>) -> Result {
    let m = c.mavlink();
    m.init(10, 8)?;
    for id in [
        mav::MSG_DATA16,
        mav::MSG_DATA32,
        mav::MSG_DATA64,
        mav::MSG_DATA96,
        mav::MSG_COMMAND_INT,
        mav::MSG_LED_CONTROL,
        mav::MSG_TUNNEL,
    ] {
        m.register_rx_msgid(id)?;
    }
    m.block_command(mav::MAV_CMD_USER_1)?;
    m.block_command(mav::MAV_CMD_USER_2)?;
    Ok(())
}

fn send_ack(c: &Context<'_>, msg: &mav::Received, cmd: &mav::CommandInt, result: u8) -> Result {
    // target_system is filled in by the host from the full (possibly 32-bit) target
    let p = mav::command_ack(cmd.command, result, 0, msg.compid);
    c.mavlink().send_chan(msg.channel, mav::MSG_COMMAND_ACK, &p, Some(msg.sysid))?;
    Ok(())
}

fn handle_command(app: &mut App<'_, '_>, c: &Context<'_>, msg: &mav::Received, cmd: &mav::CommandInt) -> Result {
    let result = match (cmd.command, cmd.param1 as i32) {
        (mav::MAV_CMD_USER_1, 0) | (mav::MAV_CMD_USER_1, 1) => {
            if c.arming().is_armed()? {
                mav::MAV_RESULT_FAILED
            } else {
                if cmd.param1 as i32 == 1 {
                    let _ = c.fs().unlink(loader::SHOW_FILE);
                }
                app.reload_show()?;
                mav::MAV_RESULT_ACCEPTED
            }
        }
        (mav::MAV_CMD_USER_1, 2) => app.handle_pyro_test(cmd.param2, cmd.param3, cmd.param4)?,
        (mav::MAV_CMD_USER_1, 4) => app.handle_light_source(cmd.param2, cmd.param3, cmd.x),
        (mav::MAV_CMD_USER_1, 3) => app.group_command(cmd)?,
        (mav::MAV_CMD_USER_2, 0) => {
            if app.configure_coordinate_system(cmd.x, cmd.y, cmd.z as i32, cmd.param4)? {
                mav::MAV_RESULT_ACCEPTED
            } else {
                mav::MAV_RESULT_FAILED
            }
        }
        (mav::MAV_CMD_USER_2, 1) => mav::MAV_RESULT_FAILED,
        _ => mav::MAV_RESULT_UNSUPPORTED,
    };
    send_ack(c, msg, cmd, result)
}

fn handle_message(app: &mut App<'_, '_>, c: &Context<'_>, msg: &mav::Received) -> Result {
    match msg.msgid {
        mav::MSG_LED_CONTROL => {
            app.handle_led_control(msg.payload());
            return Ok(());
        }
        mav::MSG_TUNNEL => {
            app.handle_tunnel(msg.payload());
            return Ok(());
        }
        _ => {}
    }
    if let Some(cmd) = msg.command_int() {
        if cmd.command == mav::MAV_CMD_USER_1 || cmd.command == mav::MAV_CMD_USER_2 {
            handle_command(app, c, msg, &cmd)?;
        }
        return Ok(());
    }
    let Some((kind, data)) = msg.data_packet() else {
        return Ok(());
    };
    if kind != mav::TYPE_GCS_TO_DRONE || data.is_empty() {
        return Ok(());
    }
    let body = &data[1..];
    match data[0] {
        mav::START_CONFIG => {
            if let Some(sc) = mav::decode_start_config(body) {
                app.handle_start_config(&sc)?;
            }
        }
        mav::TIME_AXIS_CONFIG => {
            app.handle_time_axis_config(body)?;
        }
        mav::SIMPLE_GEOFENCE_SETUP => {
            if let Some(g) = mav::decode_geofence_setup(body) {
                let pending = app.prepare_geofence(&g)?;
                let result = if pending.is_some() { mav::MAV_RESULT_ACCEPTED } else { mav::MAV_RESULT_FAILED };
                let reply = mav::ack_data(g.ack_token, result);
                c.mavlink().send_chan(msg.channel, mav::MSG_DATA16, &reply, None)?;
                if let Some(p) = pending {
                    app.apply_geofence(&p)?;
                }
            }
        }
        _ => {}
    }
    Ok(())
}

pub fn report<T>(c: &Context<'_>, what: &str, r: Result<T>) -> Result<T> {
    if let Err(e) = &r {
        if *e != Error::Stopped {
            let mut buf = [0u8; 50];
            let msg = support::fmt(&mut buf, format_args!("SkyBrush: {} failed: {:?}", what, e));
            let _ = c.gcs().send_text(MavSeverity::ERROR, msg);
        }
    }
    r
}

fn run(c: &Context<'_>) -> Result {
    // first, so GCSes asking for AUTOPILOT_VERSION on connect see it
    match c.gcs().set_extra_capabilities(monitors::SKYBRUSH_CAPABILITY) {
        Err(Error::Unavailable) => {}
        r => report(c, "capabilities", r)?,
    }
    let params = report(c, "params", params::Params::register(c))?;
    let mode = report(c, "mode", vehicle::register(c, app::MODE_NUMBER))?;
    report(c, "mavlink", setup_mavlink(c))?;
    let mut app = report(c, "init", App::new(c, params, mode))?;
    // SHOW_MODE_BOOT bit0: enter show mode once the vehicle accepts it
    let mut boot_mode_until = if app.params.get_i32(params::P::ModeBoot) & 1 != 0 {
        c.time_us() + 120_000_000
    } else {
        0
    };
    let mut boot_mode_next = 0u64;
    report(c, "telemetry", app.apply_telemetry_profile())?;
    report(c, "supervision", vehicle::start_supervision(c, app.mode()))?;
    c.gcs().send_text(MavSeverity::INFO, "SkyBrush: module started")?;
    let mut next = c.time_us() + TICK_US;
    loop {
        while let Some(raw) = c.mavlink().receive_chan()? {
            if let Some(msg) = mav::Received::parse(raw.bytes(), raw.channel) {
                handle_message(&mut app, c, &msg)?;
            }
        }
        if boot_mode_until != 0 && c.time_us() >= boot_mode_next {
            boot_mode_next = c.time_us() + 1_000_000;
            if c.time_us() > boot_mode_until || c.arming().is_armed()? || c.vehicle().set_mode(app::MODE_NUMBER)? {
                boot_mode_until = 0;
            }
        }
        report(c, "tick", app.tick())?;
        let now = c.time_us();
        if next > now {
            c.sleep_ms(((next - now + 999) / 1000) as u32)?;
        }
        next += TICK_US;
        if next + TICK_US * 5 < c.time_us() {
            next = c.time_us() + TICK_US;
        }
    }
}

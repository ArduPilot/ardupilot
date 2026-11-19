// SPDX-License-Identifier: GPL-3.0-or-later
// Custom mode controller request fixture, driven through CMT_ parameters by
// the RustCustomModeController autotest.
#![no_std]
#![no_main]

use ap_rust_sdk::{
    enums::MavSeverity, Context, CustomMode, CustomModeRequest, CustomModeStatus, Error, Result, Vector3f,
};
use core::fmt::{self, Write};
use core::num::NonZeroU16;

ap_rust_sdk::module!(run);

const MODE: u8 = 100;
const TABLE: u8 = 117;

// CMT_CMD values
const CMD_TAKEOFF: u8 = 1;
const CMD_GUIDED: u8 = 2; // wait for applied, then fly CMT_ARG m north with PVA targets
const CMD_LOITER: u8 = 3;
const CMD_LAND: u8 = 4;
const CMD_RTL: u8 = 5;
const CMD_OPTIONS: u8 = 6; // options = CMT_ARG
const CMD_ARMING_READY: u8 = 7; // ready = CMT_ARG != 0
const CMD_SUPERVISION: u8 = 8; // timeout = CMT_ARG ms
const CMD_HANG: u8 = 9; // stop keepalives
const CMD_EXIT: u8 = 10; // return, cleaning up the module
const CMD_EARLY_PVA: u8 = 11; // GUIDED request then an immediate target, only accepted once applied
const CMD_ARM: u8 = 12; // arm through the mode, applied on the main thread
const CMD_OWNED_PVA: u8 = 13; // hold position with a target only our mode accepts

struct Message {
    bytes: [u8; 100],
    length: usize,
}

impl Write for Message {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = (self.length + text.len()).min(self.bytes.len());
        self.bytes[self.length..end].copy_from_slice(&text.as_bytes()[..end - self.length]);
        self.length = end;
        Ok(())
    }
}

fn say(context: &Context<'_>, args: fmt::Arguments<'_>) -> Result {
    let mut message = Message { bytes: [0; 100], length: 0 };
    let _ = message.write_fmt(args);
    let text = core::str::from_utf8(&message.bytes[..message.length]).map_err(|_| Error::Host)?;
    context.gcs().send_text(MavSeverity::INFO, text)
}

fn param(context: &Context<'_>, name: &str) -> Result<f32> {
    context.param().get(name)?.ok_or(Error::Host)
}

struct Pva {
    generation: NonZeroU16,
    start: Vector3f,
    distance: f32,
    started_us: u64,
}

fn request(context: &Context<'_>, mode: &CustomMode, kind: CustomModeRequest, alt: f32)
           -> Result<Option<NonZeroU16>> {
    let generation = mode.set_request(context, kind, alt)?;
    say(context, format_args!("CMT: request {:?} gen {}", kind, generation.map_or(0, |g| g.get())))?;
    Ok(generation)
}

fn run(context: &Context<'_>) -> Result {
    if !(context.param().add_table(TABLE, "CMT_", 3)? && context.param().add_param(TABLE, 1, "CMD", 0.0)? &&
         context.param().add_param(TABLE, 2, "ARG", 0.0)? && context.param().add_param(TABLE, 3, "ALT", 10.0)?) {
        return Err(Error::Host);
    }
    context.param().set("CMT_CMD", 0.0)?;
    let mode = context.vehicle().register_custom_mode(MODE, "CMTest", "CMTS")?.ok_or(Error::Host)?;
    mode.set_allow_entry(context, true)?;
    say(context, format_args!("CMT: registered"))?;

    let mut last_status: Option<CustomModeStatus> = None;
    let mut keepalive = true;
    let mut guided: Option<(NonZeroU16, f32)> = None;
    let mut pva: Option<Pva> = None;
    let mut after_takeoff: Option<NonZeroU16> = None;
    loop {
        context.sleep_ms(20)?;
        if keepalive {
            mode.keepalive(context)?;
        }
        let command = param(context, "CMT_CMD")? as u8;
        if command != 0 {
            context.param().set("CMT_CMD", 0.0)?;
            let arg = param(context, "CMT_ARG")?;
            say(context, format_args!("CMT: command {} arg {}", command, arg))?;
            match command {
                CMD_TAKEOFF => { request(context, &mode, CustomModeRequest::Takeoff, param(context, "CMT_ALT")?)?; }
                CMD_GUIDED => {
                    pva = None;
                    guided = request(context, &mode, CustomModeRequest::Guided, 0.0)?.map(|g| (g, arg));
                }
                CMD_LOITER => { request(context, &mode, CustomModeRequest::Loiter, 0.0)?; }
                CMD_LAND => { request(context, &mode, CustomModeRequest::Land, 0.0)?; }
                CMD_RTL => { request(context, &mode, CustomModeRequest::Rtl, 0.0)?; }
                CMD_OPTIONS => mode.set_options(context, arg as u8)?,
                CMD_ARMING_READY => mode.set_arming_ready(context, arg != 0.0)?,
                CMD_SUPERVISION => mode.set_supervision(context, arg as u16)?,
                CMD_HANG => keepalive = false,
                CMD_EXIT => {
                    say(context, format_args!("CMT: exiting"))?;
                    return Ok(());
                }
                CMD_EARLY_PVA => {
                    pva = None;
                    guided = None;
                    let (mut rejected, mut inconsistent) = (0u8, 0u8);
                    for _ in 0..20 {
                        let generation = mode.set_request(context, CustomModeRequest::Guided, 0.0)?
                            .ok_or(Error::Host)?;
                        let mut position = context.ahrs().get_relative_position_ned_origin()?.ok_or(Error::Host)?;
                        let accepted = context.vehicle().set_target_posvelaccel_ned(
                            &mut position, &mut Vector3f::default(), &mut Vector3f::default(), false, 0.0, false,
                            0.0, false)?;
                        // the main thread may apply the request first, but never accepts a target before
                        rejected += !accepted as u8;
                        inconsistent += (accepted && !mode.status(context)?.applied(generation)) as u8;
                        context.sleep_ms(20)?;
                    }
                    say(context, format_args!("CMT: early PVA rejected {} inconsistent {}", rejected, inconsistent))?;
                }
                CMD_ARM => {
                    let requested = mode.request_arm(context, true)?;
                    let mut armed = false;
                    for _ in 0..150 {
                        mode.keepalive(context)?;
                        armed = context.arming().is_armed()?;
                        if armed {
                            break;
                        }
                        context.sleep_ms(20)?;
                    }
                    say(context, format_args!("CMT: arm request {} armed {}", requested as u8, armed as u8))?;
                }
                CMD_OWNED_PVA => {
                    let position = context.ahrs().get_relative_position_ned_origin()?.ok_or(Error::Host)?;
                    let zero = Vector3f::default();
                    let accepted = mode.set_target_posvelaccel(context, &position, &zero, &zero, false, 0.0, false,
                                                               0.0, false)?;
                    say(context, format_args!("CMT: owned PVA {}", accepted as u8))?;
                }
                _ => say(context, format_args!("CMT: unknown command"))?,
            }
        }

        let status = mode.status(context)?;
        if last_status != Some(status) {
            say(context, format_args!("CMT: gen {} app {} ok {} done {} act {} tko {} lost {}",
                                      status.generation, status.applied_generation, status.init_ok as u8,
                                      status.complete as u8, status.active as u8, status.takeoff_event as u8,
                                      status.owner_lost as u8))?;
            last_status = Some(status);
        }

        if let Some(generation) = after_takeoff {
            if status.completed(generation) {
                after_takeoff = None;
                request(context, &mode, CustomModeRequest::Loiter, 0.0)?;
            } else if status.generation != generation.get() {
                after_takeoff = None;
            }
        }
        // GCS takeoff: take off to CMT_ALT, then loiter, as the show test takeoff does
        if status.takeoff_event && mode.ack_takeoff_event(context)? {
            say(context, format_args!("CMT: takeoff event"))?;
            after_takeoff = request(context, &mode, CustomModeRequest::Takeoff, param(context, "CMT_ALT")?)?;
        }

        // PVA targets only after the GUIDED request has been applied
        if let Some((generation, distance)) = guided {
            if status.applied(generation) {
                guided = None;
                let start = context.ahrs().get_relative_position_ned_origin()?.ok_or(Error::Host)?;
                pva = Some(Pva { generation, start, distance, started_us: context.time_us() });
                say(context, format_args!("CMT: guided applied, PVA start"))?;
            } else if status.generation != generation.get() {
                guided = None;
            }
        }
        if pva.as_ref().is_some_and(|path| status.generation != path.generation.get()) {
            pva = None;
        }
        if let Some(path) = &pva {
            let elapsed = (context.time_us() - path.started_us) as f32 * 1.0e-6;
            let (offset, speed) = if elapsed < path.distance { (elapsed, 1.0) } else { (path.distance, 0.0) };
            let mut position = Vector3f { x: path.start.x + offset, y: path.start.y, z: path.start.z };
            let mut velocity = Vector3f { x: speed, y: 0.0, z: 0.0 };
            if !context.vehicle().set_target_posvelaccel_ned(&mut position, &mut velocity, &mut Vector3f::default(),
                                                             false, 0.0, false, 0.0, false)? {
                say(context, format_args!("CMT: PVA rejected"))?;
                pva = None;
            }
        }
    }
}

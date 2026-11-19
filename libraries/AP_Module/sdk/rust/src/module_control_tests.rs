// SPDX-License-Identifier: GPL-3.0-or-later
use crate::enums::{ApArmingMethod, ModeReason};
use crate::{api, Context, Error, Location, NotifyEvents, NotifyFlags};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    calls: u32,
    input: [u8; 300],
    sizes: (u32, u32),
    reply: [u8; 8],
}

unsafe extern "C" fn call(
    raw: *mut c_void,
    _: u32,
    input: *const u8,
    ni: u32,
    output: *mut u8,
    no: u32,
) -> i32 {
    let host = &mut *raw.cast::<Host>();
    host.calls += 1;
    host.sizes = (ni, no);
    host.input = [0; 300];
    if ni != 0 {
        host.input[..ni as usize].copy_from_slice(core::slice::from_raw_parts(input, ni as usize));
    }
    if no != 0 {
        core::slice::from_raw_parts_mut(output, no as usize).copy_from_slice(&host.reply[..no as usize]);
    }
    api::STATUS_OK
}

const API: api::HostApi = api::HostApi {
    version: api::ABI_VERSION,
    size: core::mem::size_of::<api::HostApi>() as u32,
    fingerprint: api::API_FINGERPRINT,
    allocate: None,
    deallocate: None,
    time_us: None,
    sleep_ms: None,
    stop_requested: None,
    text: None,
    panic: None,
    binding_call: Some(call),
};

fn host() -> Host {
    Host { calls: 0, input: [0; 300], sizes: (0, 0), reply: [0; 8] }
}

fn context(host: &mut Host) -> Context<'static> {
    Context { api: &API, raw: (host as *mut Host).cast(), _not_send_sync: PhantomData }
}

#[test]
fn mode_and_disarm_carry_reason_and_method() {
    let mut host = host();
    let context = context(&mut host);
    host.reply[0] = 1;
    assert_eq!(context.vehicle().set_mode_reason(5, ModeReason::FENCE_BREACHED), Ok(true));
    assert_eq!((host.sizes, &host.input[..5]), ((5, 1), &[5, 10, 0, 0, 0][..]));
    assert_eq!(context.arming().disarm_method(ApArmingMethod::EKFFAILSAFE, false), Ok(true));
    assert_eq!((host.sizes, &host.input[..5]), ((5, 1), &[24, 0, 0, 0, 0][..]));
    host.reply[0] = 2;
    assert_eq!(context.arming().disarm_method(ApArmingMethod::SCRIPTING, true), Err(Error::Host));
    assert_eq!(&host.input[..5], &[4, 0, 0, 0, 1]);
    host.reply[..4].copy_from_slice(&15i32.to_le_bytes());
    assert_eq!(context.arming().last_disarm_method(), Ok(ApArmingMethod::FENCEBREACH));
    assert_eq!(host.sizes, (0, 4));
    assert_eq!(ModeReason::SCRIPTING, ModeReason(32));
    assert_eq!(ModeReason::EKF_FAILSAFE, ModeReason(6));
}

#[test]
fn fence_breach_count_and_polygon_upload() {
    let mut host = host();
    let context = context(&mut host);
    host.reply[..2].copy_from_slice(&0x1234u16.to_le_bytes());
    assert_eq!(context.fence().get_breach_count(), Ok(0x1234));
    assert_eq!(host.sizes, (0, 2));

    let point = |lat: i32, lng: i32| {
        let mut bytes = [0u8; 17];
        bytes[9..13].copy_from_slice(&lat.to_le_bytes());
        bytes[13..17].copy_from_slice(&lng.to_le_bytes());
        Location::decode(&bytes)
    };
    let points = [point(-353632620, 1491652370), point(-353632620, 1491660000), point(-353640000, -1)];
    host.reply[0] = 1;
    assert_eq!(context.fence().write_inclusion_polygon(&points), Ok(true));
    assert_eq!(host.sizes, (169, 1));
    assert_eq!(host.input[0], 3);
    for (i, p) in points.iter().enumerate() {
        let offset = 1 + 8 * i;
        assert_eq!(&host.input[offset..offset + 4], &p.lat().to_le_bytes());
        assert_eq!(&host.input[offset + 4..offset + 8], &p.lng().to_le_bytes());
    }
    assert!(host.input[25..169].iter().all(|b| *b == 0));
    host.reply[0] = 0;
    assert_eq!(context.fence().write_inclusion_polygon(&points), Ok(false));
    let calls = host.calls;
    let many = [point(1, 1); crate::FENCE_POLYGON_POINTS_MAX + 1];
    assert_eq!(context.fence().write_inclusion_polygon(&points[..2]), Err(Error::Invalid));
    assert_eq!(context.fence().write_inclusion_polygon(&many), Err(Error::Invalid));
    assert_eq!(host.calls, calls);
    assert_eq!(context.fence().write_inclusion_polygon(&many[..21]), Ok(false));
    assert_eq!(host.input[0], 21);
}

#[test]
fn notify_flags_are_a_u32_bitmask() {
    let mut host = host();
    let context = context(&mut host);
    let flags = NotifyFlags::ARMED | NotifyFlags::FAILSAFE_RADIO | NotifyFlags::PRE_ARM_GPS_CHECK;
    host.reply[..4].copy_from_slice(&flags.to_le_bytes());
    assert_eq!(context.notify().get_flags(), Ok(flags));
    assert_eq!(host.sizes, (0, 4));
    assert_eq!((NotifyFlags::ARMED, NotifyFlags::FLYING, NotifyFlags::PRE_ARM_GPS_CHECK), (1, 2, 1 << 11));
}

#[test]
fn notify_event_counts_are_bytes_of_a_u64() {
    let mut host = host();
    let context = context(&mut host);
    host.reply[..8].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
    let counts = context.notify().get_event_counts().unwrap();
    assert_eq!(host.sizes, (0, 8));
    assert_eq!((counts >> NotifyEvents::COMPASS_CAL_SAVED) as u8, 2);
    assert_eq!((counts >> NotifyEvents::COMPASS_CAL_FAILED) as u8, 3);
    assert_eq!((counts >> NotifyEvents::ARMING_FAILED) as u8, 8);
}

#[test]
fn int32_parameters_register_exactly() {
    let mut host = host();
    let context = context(&mut host);
    host.reply[0] = 1;
    assert_eq!(context.param().add_param_int32(62, 3, "START_TIME", 16777216), Ok(true));
    assert_eq!(host.sizes, (23, 1));
    assert_eq!(&host.input[..2], &[62, 3]);
    assert_eq!(&host.input[2..12], b"START_TIME");
    assert_eq!(&host.input[19..23], &16777216i32.to_le_bytes());
    let calls = host.calls;
    assert_eq!(context.param().add_param_int32(62, 3, "NAME_LONGER_THAN_16", 1), Err(Error::Invalid));
    assert_eq!(host.calls, calls);
}

#[test]
fn mkdir_uses_a_bounded_path() {
    let mut host = host();
    let context = context(&mut host);
    host.reply[0] = 1;
    assert_eq!(context.fs().mkdir("collmot"), Ok(true));
    assert_eq!(host.sizes, (256, 1));
    assert_eq!(&host.input[..8], b"collmot\0");
    host.reply[0] = 0;
    assert_eq!(context.fs().mkdir("collmot"), Ok(false));
    let calls = host.calls;
    assert_eq!(context.fs().mkdir(""), Err(Error::Invalid));
    assert_eq!(host.calls, calls);
}

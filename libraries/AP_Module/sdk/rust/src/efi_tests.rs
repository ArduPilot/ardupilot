// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    invalid: bool,
}

unsafe extern "C" fn call(
    raw: *mut c_void,
    _: u32,
    input: *const u8,
    ni: u32,
    output: *mut u8,
    no: u32,
) -> i32 {
    assert!(input.is_null());
    assert_eq!((ni, no), (0, 101));
    let host = &*raw.cast::<Host>();
    let output = core::slice::from_raw_parts_mut(output, no as usize);
    output.fill(0);
    if host.invalid {
        output[4] = 4;
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

#[test]
fn efi_get_state_decodes_owned_state_and_rejects_enums() {
    let mut host = Host { invalid: false };
    let context = Context {
        api: &API,
        raw: (&mut host as *mut Host).cast(),
        _not_send_sync: PhantomData,
    };
    let state = context.efi().get_state().unwrap();
    assert_eq!(state.engine_speed_rpm, 0);
    assert_eq!(state.spark_dwell_time_ms, Some(0.0));
    assert_eq!(state.ignition_voltage, Some(0.0));
    host.invalid = true;
    assert_eq!(context.efi().get_state(), Err(Error::Host));
}

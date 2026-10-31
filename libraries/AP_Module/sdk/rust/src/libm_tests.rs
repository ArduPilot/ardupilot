// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error};
use core::{ffi::c_void, marker::PhantomData};

unsafe extern "C" fn call(
    raw: *mut c_void,
    _: u32,
    input: *const u8,
    ni: u32,
    output: *mut u8,
    no: u32,
) -> i32 {
    let status = *raw.cast::<i32>();
    if status != 0 {
        return status;
    }
    let input = core::slice::from_raw_parts(input, ni as usize);
    assert_eq!(no, 4);
    assert_eq!(&input[..4], &(-0.0f32).to_le_bytes());
    if ni == 8 {
        assert_eq!(&input[4..], &(-0.5f32).to_le_bytes());
    } else {
        assert_eq!(ni, 4);
    }
    core::ptr::copy_nonoverlapping(0x7fc01234u32.to_le_bytes().as_ptr(), output, 4);
    api::STATUS_OK
}

#[test]
fn libm_wire_and_stop() {
    let api = api::HostApi {
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
    let mut status = 0i32;
    let context = Context {
        api: &api,
        raw: (&mut status as *mut i32).cast(),
        _not_send_sync: PhantomData,
    };
    assert_eq!(context.libm().sinf(-0.0).unwrap().to_bits(), 0x7fc01234);
    assert_eq!(
        context.libm().atan2f(-0.0, -0.5).unwrap().to_bits(),
        0x7fc01234
    );
    status = api::STATUS_STOPPED;
    assert_eq!(context.libm().sinf(-0.0), Err(Error::Stopped));
    status = api::STATUS_HOST_ERROR;
    assert_eq!(context.libm().atan2f(-0.0, -0.5), Err(Error::Host));
}

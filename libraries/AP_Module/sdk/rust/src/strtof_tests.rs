// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error};
use core::{ffi::c_void, marker::PhantomData};

struct Reply {
    status: i32,
    consumed: u16,
    bits: u32,
    calls: u32,
}

unsafe extern "C" fn call(
    raw: *mut c_void,
    _: u32,
    input: *const u8,
    ni: u32,
    output: *mut u8,
    no: u32,
) -> i32 {
    let reply = &mut *raw.cast::<Reply>();
    reply.calls += 1;
    assert_eq!(ni, 256);
    assert_eq!(no, 6);
    let bytes = core::slice::from_raw_parts(input, 256);
    let end = bytes.iter().position(|&b| b == 0).unwrap();
    assert!(bytes[end..].iter().all(|&b| b == 0));
    if reply.status == 0 {
        core::ptr::copy_nonoverlapping(reply.bits.to_le_bytes().as_ptr(), output, 4);
        core::ptr::copy_nonoverlapping(reply.consumed.to_le_bytes().as_ptr(), output.add(4), 2);
    }
    reply.status
}

#[test]
fn strtof_wire_bounds_and_errors() {
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
    let mut reply = Reply {
        status: 0,
        consumed: 2,
        bits: (-0.0f32).to_bits(),
        calls: 0,
    };
    let c = Context {
        api: &api,
        raw: (&mut reply as *mut Reply).cast(),
        _not_send_sync: PhantomData,
    };
    let (n, end) = c.string().strtof("-0junk").unwrap();
    assert_eq!(n.to_bits(), (-0.0f32).to_bits());
    assert_eq!(end, 2);
    reply.bits = 0x7fc01234;
    reply.consumed = 3;
    assert_eq!(c.string().strtof("nan").unwrap().0.to_bits(), 0x7fc01234);
    reply.consumed = 0;
    reply.bits = 0;
    assert_eq!(c.string().strtof(""), Ok((0.0, 0)));
    let bytes = [b'1'; 256];
    let text = core::str::from_utf8(&bytes).unwrap();
    reply.consumed = 255;
    assert_eq!(c.string().strtof(&text[..255]), Ok((0.0, 255)));
    let calls = reply.calls;
    assert_eq!(c.string().strtof(text), Err(Error::Invalid));
    assert_eq!(c.string().strtof("1\0junk"), Err(Error::Invalid));
    assert_eq!(reply.calls, calls);
    assert_eq!(c.string().strtof("1"), Err(Error::Host)); // impossible host offset
    reply.status = api::STATUS_STOPPED;
    assert_eq!(c.string().strtof("1"), Err(Error::Stopped));
    reply.status = api::STATUS_HOST_ERROR;
    assert_eq!(c.string().strtof("1"), Err(Error::Host));
}

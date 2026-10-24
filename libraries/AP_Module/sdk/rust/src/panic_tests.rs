// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{api, panic_prefix, panic_text, Context, Error};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    calls: u32,
    status: i32,
    input: [u8; 585],
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
    assert_eq!((ni, no), (585, 258));
    host.input
        .copy_from_slice(core::slice::from_raw_parts(input, ni as usize));
    host.calls += 1;
    if host.status == 0 {
        core::ptr::copy_nonoverlapping([2, 0, b'o', b'k', 0].as_ptr(), output, 5);
    }
    host.status
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

fn context(host: &mut Host) -> Context<'_> {
    Context {
        api: &API,
        raw: (host as *mut Host).cast(),
        _not_send_sync: PhantomData,
    }
}

#[test]
fn panic_uses_firmware_printf_for_literal_and_location() {
    let mut host = Host {
        calls: 0,
        status: 0,
        input: [0; 585],
    };
    assert_eq!(
        panic_text(
            &context(&mut host),
            "failure %s",
            Some(("module.rs", 123, 45))
        )
        .unwrap()
        .as_str(),
        "ok"
    );
    assert_eq!(&host.input[..15], b"%s at %s:%u:%u\0");
    assert_eq!(&host.input[256..261], &[4, 6, 6, 2, 2]);
    assert_eq!(&host.input[329..350], b"failure %s\0module.rs\0");
    assert_eq!(&host.input[281..289], &123u64.to_le_bytes());
    assert_eq!(&host.input[289..297], &45u64.to_le_bytes());
    assert_eq!(
        panic_text(&context(&mut host), "literal", None)
            .unwrap()
            .as_str(),
        "ok"
    );
    assert_eq!(&host.input[..3], b"%s\0");
    assert_eq!(host.calls, 2);
}

#[test]
fn panic_fields_are_bounded_at_utf8_boundaries() {
    assert_eq!(panic_prefix("aé", 2), "a");
    let message = "é".repeat(200);
    let filename = "€".repeat(200) + "/module.rs";
    let mut host = Host {
        calls: 0,
        status: 0,
        input: [0; 585],
    };
    assert_eq!(
        panic_text(
            &context(&mut host),
            &message,
            Some((&filename, u32::MAX, u32::MAX))
        )
        .unwrap()
        .as_str(),
        "ok"
    );
    let pool = core::str::from_utf8(&host.input[329..]).unwrap();
    let mut fields = pool.split('\0');
    assert_eq!(fields.next().unwrap().len(), 96);
    let file = fields.next().unwrap();
    assert!(file.len() <= 128 && file.ends_with("/module.rs"));
}

#[test]
fn panic_formatting_can_fail_without_panicking() {
    let mut host = Host {
        calls: 0,
        status: api::STATUS_STOPPED,
        input: [0; 585],
    };
    assert!(matches!(
        panic_text(&context(&mut host), "literal", Some(("module.rs", 1, 2))),
        Err(Error::Stopped)
    ));
    host.status = api::STATUS_UNAVAILABLE;
    assert!(matches!(
        panic_text(&context(&mut host), "literal", None),
        Err(Error::Unavailable)
    ));
    assert!(matches!(
        panic_text(&context(&mut host), "embedded\0NUL", None),
        Err(Error::Invalid)
    ));
    assert_eq!(host.calls, 2);
}

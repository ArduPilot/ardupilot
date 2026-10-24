// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    mode: u8,
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
    let host = &mut *raw.cast::<Host>();
    host.calls += 1;
    assert_eq!((ni, no), (262, 259));
    let input = core::slice::from_raw_parts(input, ni as usize);
    assert_eq!(&input[..4], b"file");
    assert!(input[4..256].iter().all(|b| *b == 0));
    assert_eq!(&input[256..], &[7, 0, 0, 0, 8, 0]);
    let output = core::slice::from_raw_parts_mut(output, no as usize);
    output.fill(0);
    output[0] = 1;
    match host.mode {
        0 => {
            output[1] = 3;
            output[3..6].copy_from_slice(b"abc");
        }
        1 => output[0] = 0,
        2 => {}
        3 => {
            output.fill(255);
            return api::STATUS_HOST_ERROR;
        }
        4 => output[1] = 9,
        _ => output[0] = 2,
    }
    api::STATUS_OK
}

#[test]
fn bounded_file_packets_and_unchanged_buffer_on_errors() {
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
    let mut host = Host { mode: 0, calls: 0 };
    let context = Context {
        api: &api,
        raw: (&mut host as *mut Host).cast(),
        _not_send_sync: PhantomData,
    };
    let mut buffer = [0x55u8; 8];
    assert_eq!(context.file().read_at("file", 7, &mut buffer), Ok(Some(3)));
    assert_eq!(&buffer, b"abcUUUUU");
    for (mode, expected) in [
        (1, Ok(None)),
        (2, Ok(Some(0))),
        (3, Err(Error::Host)),
        (4, Err(Error::Host)),
        (5, Err(Error::Host)),
    ] {
        unsafe {
            (*context.raw.cast::<Host>()).mode = mode;
        }
        let before = buffer;
        assert_eq!(context.file().read_at("file", 7, &mut buffer), expected);
        assert_eq!(buffer, before);
    }
    let before = host.calls;
    for path in ["", "@", "/@PARAM", "//@SYS", "embedded\0nul"] {
        assert_eq!(
            context.file().read_at(path, 0, &mut buffer),
            Err(Error::Invalid)
        );
    }
    let long = [b'x'; 256];
    assert_eq!(
        context.file().read_at(core::str::from_utf8(&long).unwrap(), 0, &mut buffer),
        Err(Error::Invalid)
    );
    for offset in [i32::MAX as u32, i32::MAX as u32 - 6, u32::MAX] {
        assert_eq!(
            context.file().read_at("file", offset, &mut buffer),
            Err(Error::Invalid)
        );
    }
    assert_eq!(
        context.file().read_at("file", 0, &mut []),
        Err(Error::Invalid)
    );
    assert_eq!(
        context.file().read_at("file", 0, &mut [0u8; 257]),
        Err(Error::Invalid)
    );
    assert_eq!(host.calls, before);
}

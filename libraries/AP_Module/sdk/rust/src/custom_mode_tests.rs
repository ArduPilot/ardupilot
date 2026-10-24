// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error};
use core::{ffi::c_void, marker::PhantomData};

unsafe extern "C" fn call(
    _: *mut c_void,
    _: u32,
    input: *const u8,
    ni: u32,
    output: *mut u8,
    no: u32,
) -> i32 {
    let input = core::slice::from_raw_parts(input, ni as usize);
    match (ni, no) {
        (40, 4) => {
            assert_eq!(input[0], 27);
            assert_eq!(input[1], 9);
            assert_eq!(&input[2..11], b"Rust mode");
            assert!(input[11..34].iter().all(|byte| *byte == 0));
            assert_eq!(input[34], 4);
            assert_eq!(&input[35..39], b"RUST");
            assert_eq!(input[39], 0);
            core::slice::from_raw_parts_mut(output, no as usize)
                .copy_from_slice(&0x12345678u32.to_le_bytes());
        }
        (4, 1) => {
            assert_eq!(input, &0x12345678u32.to_le_bytes());
            *output = 1;
        }
        (5, 0) => {
            assert_eq!(&input[..4], &0x12345678u32.to_le_bytes());
            assert_eq!(input[4], 0);
        }
        _ => panic!("unexpected custom mode wire shape"),
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
fn custom_mode_uses_bounded_names_and_opaque_handles() {
    let context = Context {
        api: &API,
        raw: core::ptr::NonNull::<u8>::dangling().as_ptr().cast(),
        _not_send_sync: PhantomData,
    };
    let mode = context.vehicle().register_custom_mode(27, "Rust mode", "RUST").unwrap().unwrap();
    assert_eq!(mode.allow_entry(&context), Ok(true));
    assert_eq!(mode.set_allow_entry(&context, false), Ok(()));
    assert!(matches!(context.vehicle().register_custom_mode(27, "", "RUST"), Err(Error::Invalid)));
    assert!(matches!(context.vehicle().register_custom_mode(27, "Rust mode", "TOOLONG"),
                     Err(Error::Invalid)));
    assert!(matches!(context.vehicle().register_custom_mode(27, "bad\0name", "RUST"),
                     Err(Error::Invalid)));
}

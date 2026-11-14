// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, CameraInformation, Context, Error};
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
        (1, 31) => {
            assert_eq!(input, &[3]);
            let output = core::slice::from_raw_parts_mut(output, no as usize);
            output[0] = 1;
            output[1..3].copy_from_slice(&513u16.to_le_bytes());
            output[3] = 1;
            output[4] = 2;
            output[5..9].copy_from_slice(&(-0.0f32).to_le_bytes());
            output[9] = 4;
            output[10..14].copy_from_slice(&12.5f32.to_le_bytes());
            output[14] = 2;
            for (part, value) in output[15..].chunks_exact_mut(4).zip([0.1f32, 0.2, 0.8, 0.9]) {
                part.copy_from_slice(&value.to_le_bytes());
            }
        }
        (231, 0) => {
            assert_eq!(input[0], 4);
            assert_eq!(&input[1..5], &0x12345678u32.to_le_bytes());
            assert_eq!(&input[27..34], b"Vendor\0");
            assert!(input[34..59].iter().all(|byte| *byte == 0));
            assert_eq!(&input[59..65], b"Model\0");
            assert_eq!(&input[91..100], b"file.xml\0");
        }
        _ => panic!("unexpected camera wire shape"),
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
fn camera_values_have_explicit_bounded_wire_formats() {
    let context = Context {
        api: &API,
        raw: core::ptr::NonNull::<u8>::dangling().as_ptr().cast(),
        _not_send_sync: PhantomData,
    };
    let state = context.camera().get_state(3).unwrap().unwrap();
    assert_eq!(state.take_picture_increment, 513);
    assert!(state.recording_video);
    assert_eq!(state.tracking_type, 2);
    assert_eq!(state.tracking_point_2.x, 0.8);

    let mut information = CameraInformation::default();
    information.firmware_version = 0x12345678;
    assert_eq!(information.set_vendor_name("Vendor"), Ok(()));
    assert_eq!(information.set_model_name("Model"), Ok(()));
    assert_eq!(information.set_definition_uri("file.xml"), Ok(()));
    assert_eq!(context.camera().set_information(4, &information), Ok(()));
    assert_eq!(information.set_vendor_name(&"x".repeat(32)), Err(Error::Invalid));
}

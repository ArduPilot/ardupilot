// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, CustomModeRequest, Error, Vector3f, CUSTOM_MODE_ARMING_GATED, CUSTOM_MODE_EKF_FS_DISARM};
use core::{ffi::c_void, marker::PhantomData, num::NonZeroU16};
use core::sync::atomic::{AtomicU8, Ordering};

// expected control call, selected by the test because unpatched slots are identical
static EXPECT: AtomicU8 = AtomicU8::new(0);

unsafe extern "C" fn call(
    _: *mut c_void,
    _: u32,
    input: *const u8,
    ni: u32,
    output: *mut u8,
    no: u32,
) -> i32 {
    let input = core::slice::from_raw_parts(input, ni as usize);
    let expect = EXPECT.load(Ordering::Relaxed);
    if expect != 0 {
        assert_eq!(&input[..4], &0x12345678u32.to_le_bytes());
        assert_eq!(output.is_null(), no == 0);
        let output: &mut [u8] = if no == 0 { &mut [] } else { core::slice::from_raw_parts_mut(output, no as usize) };
        match (expect, ni, no) {
            (1, 9, 2) => {
                assert_eq!(input[4], CustomModeRequest::Takeoff as u8);
                assert_eq!(&input[5..9], &2.5f32.to_le_bytes());
                output.copy_from_slice(&7u16.to_le_bytes());
            }
            (2, 9, 2) => output.copy_from_slice(&[0, 0]),
            (3, 4, 10) => output.copy_from_slice(&[2, 7, 0, 7, 0, 1, 1, 1, 0, 0]),
            (4, 5, 0) => assert_eq!(input[4], CUSTOM_MODE_EKF_FS_DISARM | CUSTOM_MODE_ARMING_GATED),
            (5, 5, 0) => assert_eq!(input[4], 1),
            (6, 6, 0) => assert_eq!(&input[4..6], &1000u16.to_le_bytes()),
            (7, 4, 0) => {}
            (8, 4, 1) => output[0] = 1,
            (9, 4, 10) => output.copy_from_slice(&[9, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
            (10, 5, 1) => {
                assert!(input[4] <= 1);
                output[0] = 1;
            }
            (11, 51, 1) => {
                for (i, v) in [1.0f32, 2.0, -3.0, 0.5, 0.25, 0.0, 0.0, 0.0, 0.1].iter().enumerate() {
                    assert_eq!(&input[4 + 4 * i..8 + 4 * i], &v.to_le_bytes());
                }
                assert_eq!(input[40], 1);
                assert_eq!(&input[41..45], &90.0f32.to_le_bytes());
                assert_eq!(input[45], 1);
                assert_eq!(&input[46..50], &5.0f32.to_le_bytes());
                assert_eq!(input[50], 0);
                output[0] = 0;
            }
            _ => panic!("unexpected custom mode control wire shape"),
        }
        return api::STATUS_OK;
    }
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

    let expect = |n| EXPECT.store(n, Ordering::Relaxed);
    expect(1);
    let generation = mode.set_request(&context, CustomModeRequest::Takeoff, 2.5).unwrap().unwrap();
    assert_eq!(generation.get(), 7);
    expect(2);
    assert_eq!(mode.set_request(&context, CustomModeRequest::Takeoff, 2.5), Ok(None));
    assert_eq!(mode.set_request(&context, CustomModeRequest::Takeoff, f32::NAN), Err(Error::Invalid));
    expect(3);
    let status = mode.status(&context).unwrap();
    assert_eq!(status.request, CustomModeRequest::Land);
    assert!(status.applied(generation) && status.completed(generation) && status.active);
    assert!(!status.takeoff_event && !status.owner_lost);
    assert!(!status.applied(NonZeroU16::new(8).unwrap()));
    expect(4);
    assert_eq!(mode.set_options(&context, CUSTOM_MODE_EKF_FS_DISARM | CUSTOM_MODE_ARMING_GATED), Ok(()));
    assert_eq!(mode.set_options(&context, 8), Err(Error::Invalid));
    expect(5);
    assert_eq!(mode.set_arming_ready(&context, true), Ok(()));
    expect(6);
    assert_eq!(mode.set_supervision(&context, 1000), Ok(()));
    expect(7);
    assert_eq!(mode.keepalive(&context), Ok(()));
    expect(8);
    assert_eq!(mode.ack_takeoff_event(&context), Ok(true));
    expect(9);
    assert_eq!(mode.status(&context), Err(Error::Host));
    expect(10);
    assert_eq!(mode.request_arm(&context, true), Ok(true));
    assert_eq!(mode.request_arm(&context, false), Ok(true));
    expect(11);
    let v = |x, y, z| Vector3f { x, y, z };
    let (pos, vel, acc) = (v(1.0, 2.0, -3.0), v(0.5, 0.25, 0.0), v(0.0, 0.0, 0.1));
    assert_eq!(mode.set_target_posvelaccel(&context, &pos, &vel, &acc, true, 90.0, true, 5.0, false), Ok(false));
    assert_eq!(mode.set_target_posvelaccel(&context, &v(f32::NAN, 0.0, 0.0), &vel, &acc, true, 90.0, true, 5.0, false),
               Err(Error::Invalid));
    expect(0);
}

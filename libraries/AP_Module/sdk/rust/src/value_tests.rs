// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error, Location, MissionItem, Vector2f};
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
    // change_alt_frame: false, but the host modified receiver state.
    assert_eq!((ni, no), (21, 18));
    let input = core::slice::from_raw_parts(input, ni as usize);
    let output = core::slice::from_raw_parts_mut(output, no as usize);
    output[0] = 0;
    output[1..].copy_from_slice(&input[..17]);
    output[10..14].copy_from_slice(&123i32.to_le_bytes());
    api::STATUS_OK
}

#[test]
fn complete_value_encoding() {
    assert_eq!(Location::default().encode(), [0; 17]);
    assert_eq!(MissionItem::default().encode(), [0; 38]);
    assert_eq!(Vector2f::default().encode(), [0; 8]);
    let mut location = [0u8; 17];
    location[..5].copy_from_slice(&[1, 1, 0, 1, 1]);
    location[5..9].copy_from_slice(&(-9000i32).to_le_bytes());
    location[9..13].copy_from_slice(&1900000000i32.to_le_bytes());
    assert_eq!(Location::decode(&location).encode(), location);
    let mut mission = [0u8; 38];
    for (i, byte) in mission.iter_mut().enumerate() {
        *byte = (i as u8).wrapping_mul(17).wrapping_add(139);
    }
    let mut item = MissionItem::decode(&mission);
    assert_eq!(item.encode(), mission);
    item.command = 177;
    assert_eq!(&item.encode()[32..], &mission[32..]);
}

#[test]
fn copyback_includes_false_but_not_host_errors() {
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
    let mut bytes = [0; 17];
    bytes[1] = 1; // private loiter_ccw
    let mut location = Location::decode(&bytes);
    assert_eq!(
        location.change_alt_frame(&context, crate::enums::LocationAltFrame(0)),
        Ok(false)
    );
    assert_eq!(location.lat(), 123);
    assert_eq!(location.encode()[1], 1);
    let before = location.encode();
    unsafe {
        context.raw.cast::<i32>().write(api::STATUS_BUSY);
    }
    assert_eq!(
        location.change_alt_frame(&context, crate::enums::LocationAltFrame(0)),
        Err(Error::Busy)
    );
    assert_eq!(location.encode(), before);
}

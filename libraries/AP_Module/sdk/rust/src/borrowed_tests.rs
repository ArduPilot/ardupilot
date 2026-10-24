// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error, RcChannel};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    token: u32,
    status: i32,
    pwm: u16,
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
    if host.status != 0 {
        return host.status;
    }
    let input = core::slice::from_raw_parts(input, ni as usize);
    let output = core::slice::from_raw_parts_mut(output, no as usize);
    match (ni, no) {
        (1, 4) => {
            assert_eq!(input, &[9]);
            output.copy_from_slice(&host.token.to_le_bytes());
        }
        (4, 4) => {
            assert_eq!(input, &9u32.to_le_bytes());
            output.copy_from_slice(&0.25f32.to_le_bytes());
        }
        (6, 0) => {
            assert_eq!(&input[..4], &9u32.to_le_bytes());
            host.pwm = u16::from_le_bytes(input[4..].try_into().unwrap());
        }
        _ => panic!("unexpected handle packet"),
    }
    api::STATUS_OK
}

#[test]
fn rc_handle_encoding_and_errors() {
    assert_eq!(core::mem::size_of::<RcChannel>(), 4);
    assert_eq!(core::mem::size_of::<Option<RcChannel>>(), 4);
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
    let mut host = Host {
        token: 9,
        status: 0,
        pwm: 0,
    };
    let context = Context {
        api: &api,
        raw: (&mut host as *mut Host).cast(),
        _not_send_sync: PhantomData,
    };
    let channel = context.rc().get_channel(9).unwrap().unwrap();
    assert_eq!(channel.norm_input(&context), Ok(0.25));
    assert_eq!(channel.set_override(&context, 1600), Ok(()));
    assert_eq!(host.pwm, 1600);
    host.token = 0;
    assert_eq!(context.rc().get_channel(9), Ok(None));
    // Existing token remains valid independently of a later query result.
    assert_eq!(channel.norm_input(&context), Ok(0.25));
    host.status = api::STATUS_BUSY;
    assert_eq!(channel.norm_input(&context), Err(Error::Busy));
    host.status = api::STATUS_STOPPED;
    assert_eq!(channel.set_override(&context, 0), Err(Error::Stopped));
}

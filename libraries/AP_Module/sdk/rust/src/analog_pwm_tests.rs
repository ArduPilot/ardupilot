// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    step: u8,
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
    let input = if ni == 0 { &[] } else { core::slice::from_raw_parts(input, ni as usize) };
    let output = if no == 0 { &mut [] } else {
        core::slice::from_raw_parts_mut(output, no as usize)
    };
    match host.step {
        0 | 5 => {
            assert_eq!((ni, no), (0, 4));
            let token = if host.step == 0 { 0x11u32 } else { 0x22u32 };
            output.copy_from_slice(&token.to_le_bytes());
        }
        1 => {
            assert_eq!((ni, no), (5, 1));
            assert_eq!(input, &[0x11, 0, 0, 0, 13]);
            output[0] = 1;
        }
        2 | 3 | 4 => {
            assert_eq!((ni, no), (4, 4));
            assert_eq!(input, &[0x11, 0, 0, 0]);
            let value = match host.step { 2 => 1.25f32, 3 => 2.5, _ => 0.5 };
            output.copy_from_slice(&value.to_le_bytes());
        }
        6 => {
            assert_eq!((ni, no), (5, 1));
            assert_eq!(input, &[0x22, 0, 0, 0, 50]);
            output[0] = 1;
        }
        7 | 8 => {
            assert_eq!((ni, no), (4, 2));
            assert_eq!(input, &[0x22, 0, 0, 0]);
            let value = if host.step == 7 { 1500u16 } else { 1600u16 };
            output.copy_from_slice(&value.to_le_bytes());
        }
        9 => {
            assert_eq!((ni, no), (4, 0));
            assert_eq!(input, &[0x22, 0, 0, 0]);
        }
        _ => panic!("unexpected analog/PWM call"),
    }
    host.step += 1;
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
fn analog_and_pwm_sources_use_typed_opaque_handles() {
    let mut host = Host { step: 0 };
    let context = Context {
        api: &API,
        raw: (&mut host as *mut Host).cast(),
        _not_send_sync: PhantomData,
    };
    let analog = context.analog().channel().unwrap().unwrap();
    assert_eq!(analog.set_pin(&context, 13), Ok(true));
    assert_eq!(analog.voltage_average(&context), Ok(1.25));
    assert_eq!(analog.voltage_latest(&context), Ok(2.5));
    assert_eq!(analog.voltage_average_ratiometric(&context), Ok(0.5));

    let pwm = context.pwm_source().open().unwrap().unwrap();
    assert_eq!(pwm.set_pin(&context, 50), Ok(true));
    assert_eq!(pwm.get_pwm_us(&context), Ok(1500));
    assert_eq!(pwm.get_pwm_avg_us(&context), Ok(1600));
    assert_eq!(pwm.close(&context), Ok(()));
    assert_eq!(host.step, 10);
}

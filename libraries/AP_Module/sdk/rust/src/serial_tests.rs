// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error, SerialFlowControl, SerialParity, SerialStopBits};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    step: u8,
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
    let input = core::slice::from_raw_parts(input, ni as usize);
    match host.step {
        0 => {
            assert_eq!((input, no), (&[2][..], 4));
            core::slice::from_raw_parts_mut(output, 4).copy_from_slice(&7u32.to_le_bytes());
        }
        1 => {
            assert_eq!((input, no), (&[28, 3, 0][..], 4));
            core::slice::from_raw_parts_mut(output, 4).copy_from_slice(&0x811c0007u32.to_le_bytes());
        }
        2 => assert_eq!((input, no), (&[7, 0, 0, 0, 1, 0, 194, 1, 0][..], 0)),
        3 => assert_eq!((input, no), (&[7, 0, 0, 0, 2][..], 0)),
        4 => assert_eq!((input, no), (&[7, 0, 0, 0, 2][..], 0)),
        5 => assert_eq!((input, no), (&[7, 0, 0, 0, 1][..], 0)),
        6 => assert_eq!((input, no), (&[7, 0, 0, 0, 3][..], 0)),
        7 => {
            assert_eq!((input, no), (&[7, 0, 0, 0][..], 4));
            core::slice::from_raw_parts_mut(output, 4).copy_from_slice(&12u32.to_le_bytes());
        }
        8 => {
            assert_eq!((ni, no), (262, 2));
            assert_eq!(&input[..9], &[7, 0, 0, 0, 3, 0, b'a', b'b', b'c']);
            assert!(input[9..].iter().all(|byte| *byte == 0));
            core::slice::from_raw_parts_mut(output, 2).copy_from_slice(&3u16.to_le_bytes());
        }
        9 => {
            assert_eq!((input, no), (&[7, 0, 0, 0, 4, 0][..], 258));
            let output = core::slice::from_raw_parts_mut(output, 258);
            output.fill(0);
            output[..2].copy_from_slice(&3u16.to_le_bytes());
            output[2..5].copy_from_slice(b"xyz");
        }
        10 => {
            assert_eq!((ni, no), (262, 2));
            assert_eq!(&input[..7], &[7, 0, 0, 0, 1, 0, 0x55]);
            core::slice::from_raw_parts_mut(output, 2).copy_from_slice(&1u16.to_le_bytes());
        }
        11 => {
            assert_eq!((input, no), (&[7, 0, 0, 0, 1, 0][..], 258));
            let output = core::slice::from_raw_parts_mut(output, 258);
            output.fill(0);
            output[..2].copy_from_slice(&1u16.to_le_bytes());
            output[2] = 0x66;
        }
        _ => return api::STATUS_HOST_ERROR,
    }
    host.step += 1;
    api::STATUS_OK
}

fn context(host: &mut Host) -> Context<'_> {
    static API: api::HostApi = api::HostApi {
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
    Context {
        api: &API,
        raw: (host as *mut Host).cast(),
        _not_send_sync: PhantomData,
    }
}

#[test]
fn typed_serial_api_has_bounded_canonical_packets() {
    let mut host = Host { step: 0, calls: 0 };
    let context = context(&mut host);
    let port = context.serial().find_serial(2).unwrap().unwrap();
    assert!(context.serial().find_simulated_device(28, 3).unwrap().is_some());
    assert_eq!(context.serial().find_simulated_device(128, 0), Err(Error::Invalid));
    port.begin(&context, Some(115200)).unwrap();
    port.configure_parity(&context, SerialParity::Even).unwrap();
    port.set_stop_bits(&context, SerialStopBits::Two).unwrap();
    port.set_unbuffered_writes(&context, true).unwrap();
    port.set_flow_control(&context, SerialFlowControl::RtsDe).unwrap();
    assert_eq!(port.available(&context), Ok(12));
    assert_eq!(port.write(&context, b"abc"), Ok(3));
    let mut bytes = [0x55; 4];
    assert_eq!(port.read(&context, &mut bytes), Ok(3));
    assert_eq!(&bytes, b"xyzU");
    assert_eq!(port.write_byte(&context, 0x55), Ok(true));
    assert_eq!(port.read_byte(&context), Ok(Some(0x66)));
    assert_eq!(port.write(&context, &[]), Ok(0));
    assert_eq!(port.read(&context, &mut []), Ok(0));
    assert_eq!(port.write(&context, &[0; 257]), Err(Error::Invalid));
    assert_eq!(port.read(&context, &mut [0; 257]), Err(Error::Invalid));
    assert_eq!(port.begin(&context, Some(0)), Err(Error::Invalid));
    drop(context);
    assert_eq!(host.calls, 12);
}

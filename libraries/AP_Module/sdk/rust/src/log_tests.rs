// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error, LogValue as V};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    calls: u32,
    status: i32,
    packet: [u8; 358],
}

unsafe extern "C" fn call(
    raw: *mut c_void,
    _: u32,
    input: *const u8,
    ni: u32,
    _: *mut u8,
    no: u32,
) -> i32 {
    assert_eq!((ni, no), (358, 0));
    let host = &mut *raw.cast::<Host>();
    host.calls += 1;
    host.packet
        .copy_from_slice(core::slice::from_raw_parts(input, ni as usize));
    host.status
}

#[test]
fn typed_log_values_and_bounds() {
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
        calls: 0,
        status: 0,
        packet: [0; 358],
    };
    let context = Context {
        api: &api,
        raw: (&mut host as *mut Host).cast(),
        _not_send_sync: PhantomData,
    };
    context
        .logger().write(
            "TEST",
            "A,B,C,D",
            Some(("-m--", "-0--")),
            &[
                V::I8(-8),
                V::Float(f32::from_bits(0x7fc12345)),
                V::U64(0xfedcba9876543210),
                V::Text4(b"a\0x"),
            ],
        )
        .unwrap();
    assert_eq!(&host.packet[..5], b"TEST\0");
    assert_eq!(&host.packet[63..79], b"bfQn\0\0\0\0\0\0\0\0\0\0\0\0");
    assert_eq!(&host.packet[79..84], b"-m--\0");
    assert_eq!(&host.packet[95..100], b"-0--\0");
    assert_eq!(&host.packet[111..114], &[1, 17, 0]);
    assert_eq!(host.packet[114], 248);
    assert_eq!(&host.packet[115..119], &0x7fc12345u32.to_le_bytes());
    assert_eq!(&host.packet[119..127], &0xfedcba9876543210u64.to_le_bytes());
    assert_eq!(&host.packet[127..131], b"a\0x\0");
    assert!(host.packet[131..].iter().all(|b| *b == 0));
    context
        .logger().write(
            "MAX",
            "A,B,C,D,E,F,G",
            None,
            &[
                V::Text64(&[1; 64]),
                V::Text64(&[2; 64]),
                V::Text64(&[3; 64]),
                V::Text16(&[4; 16]),
                V::Text16(&[5; 16]),
                V::Text16(&[6; 16]),
                V::I32(-1),
            ],
        )
        .unwrap();
    assert_eq!(&host.packet[111..114], &[0, 244, 0]);
    let before = host.calls;
    for name in ["", "LONG5", "a\0b"] {
        assert_eq!(
            context.logger().write(name, "V", None, &[V::U8(1)]),
            Err(Error::Invalid)
        );
    }
    assert_eq!(
        context.logger().write("TEST", "V", None, &[]),
        Err(Error::Invalid)
    );
    assert_eq!(
        context.logger().write("TEST", "V", None, &[V::U8(1); 16]),
        Err(Error::Invalid)
    );
    assert_eq!(
        context.logger().write("TEST", "V", None, &[V::Text4(b"long5")]),
        Err(Error::Invalid)
    );
    assert_eq!(
        context.logger().write("TEST", "A,B,C,D", None, &[V::Text64(&[0; 64]); 4]),
        Err(Error::Invalid)
    );
    assert_eq!(
        context.logger().write("TEST", "V", Some(("mm", "0")), &[V::U8(1)]),
        Err(Error::Invalid)
    );
    assert_eq!(host.calls, before);
    unsafe {
        (*context.raw.cast::<Host>()).status = api::STATUS_STOPPED;
    }
    assert_eq!(
        context.logger().write("TEST", "V", None, &[V::U8(1)]),
        Err(Error::Stopped)
    );
}

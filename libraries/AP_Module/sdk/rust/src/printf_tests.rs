// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error, PrintfArg};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    calls: u32,
    status: i32,
    input: [u8; 585],
    output: [u8; 258],
}

impl Host {
    fn new() -> Self {
        let mut host = Self {
            calls: 0,
            status: 0,
            input: [0; 585],
            output: [0; 258],
        };
        host.output[..5].copy_from_slice(&[2, 0, b'o', b'k', 0]);
        host
    }
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
    core::ptr::copy_nonoverlapping(host.output.as_ptr(), output, no as usize);
    host.calls += 1;
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
fn printf_packet_is_typed_and_pointer_free() {
    let mut host = Host::new();
    let args = [
        (-7i32).into(),
        42u32.into(),
        i64::MIN.into(),
        u64::MAX.into(),
        (-0.0f64).into(),
        "é".into(),
        "two".into(),
    ];
    let text = context(&mut host)
        .string()
        .format_printf("%d %u %lld %llu %f %s %s", &args)
        .unwrap();
    assert_eq!(text.as_str(), "ok");
    assert_eq!(host.input[256], 7);
    assert_eq!(&host.input[257..264], &[1, 2, 3, 4, 5, 6, 6]);
    for (index, expected) in [
        u32::MAX as u64 - 6,
        42,
        i64::MIN as u64,
        u64::MAX,
        (-0.0f64).to_bits(),
        0,
        3,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(
            u64::from_le_bytes(
                host.input[265 + index * 8..273 + index * 8]
                    .try_into()
                    .unwrap()
            ),
            expected
        );
    }
    assert_eq!(&host.input[329..336], "é\0two\0".as_bytes());
    assert_eq!(host.calls, 1);
}

#[test]
fn printf_rejects_bad_input_before_host_call() {
    let mut host = Host::new();
    let full = core::str::from_utf8(&[b'x'; 256]).unwrap();
    for format in [full, "bad\0format"] {
        assert!(matches!(
            context(&mut host).string().format_printf(format, &[]),
            Err(Error::Invalid)
        ));
    }
    assert!(matches!(
        context(&mut host)
            .string()
            .format_printf("%s", &[full.into()]),
        Err(Error::Invalid)
    ));
    assert!(matches!(
        context(&mut host)
            .string()
            .format_printf("%s", &["bad\0text".into()]),
        Err(Error::Invalid)
    ));
    assert!(matches!(
        context(&mut host)
            .string()
            .format_printf("%s%s", &[full[..255].into(), "".into()]),
        Err(Error::Invalid)
    ));
    let too_many = [
        PrintfArg::Int32(0),
        PrintfArg::Int32(0),
        PrintfArg::Int32(0),
        PrintfArg::Int32(0),
        PrintfArg::Int32(0),
        PrintfArg::Int32(0),
        PrintfArg::Int32(0),
        PrintfArg::Int32(0),
        PrintfArg::Int32(0),
    ];
    assert!(matches!(
        context(&mut host).string().format_printf("", &too_many),
        Err(Error::Invalid)
    ));
    assert_eq!(host.calls, 0);
    assert_eq!(
        context(&mut host)
            .string()
            .format_printf("%s", &[full[..255].into()])
            .unwrap()
            .as_str(),
        "ok"
    );
}

#[test]
fn printf_checks_host_errors_and_output() {
    let mut host = Host::new();
    for (status, expected) in [
        (api::STATUS_STOPPED, Error::Stopped),
        (api::STATUS_INVALID, Error::Invalid),
        (api::STATUS_UNAVAILABLE, Error::Unavailable),
        (api::STATUS_HOST_ERROR, Error::Host),
    ] {
        host.status = status;
        assert!(
            matches!(context(&mut host).string().format_printf("", &[]), Err(error) if error == expected)
        );
    }
    host.status = 0;
    host.output[..2].copy_from_slice(&256u16.to_le_bytes());
    assert!(matches!(
        context(&mut host).string().format_printf("", &[]),
        Err(Error::Host)
    ));
    host.output[0] = 2;
    host.output[1] = 0;
    host.output[4] = b'x';
    assert!(matches!(
        context(&mut host).string().format_printf("", &[]),
        Err(Error::Host)
    ));
    host.output[4] = 0;
    host.output[2] = 0xff;
    assert!(matches!(
        context(&mut host).string().format_printf("", &[]),
        Err(Error::Invalid)
    ));
    host.output[2] = 0;
    assert!(matches!(
        context(&mut host).string().format_printf("", &[]),
        Err(Error::Invalid)
    ));
}

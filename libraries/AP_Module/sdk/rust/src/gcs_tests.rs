// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{api, enums::MavSeverity, Context, Error};
use core::{ffi::c_void, fmt, marker::PhantomData};

struct Host {
    calls: u32,
    status: i32,
    packet: [u8; 260],
}

unsafe extern "C" fn call(
    raw: *mut c_void,
    _: u32,
    input: *const u8,
    input_size: u32,
    _: *mut u8,
    output_size: u32,
) -> i32 {
    let host = &mut *raw.cast::<Host>();
    assert_eq!(input_size, 260);
    assert_eq!(output_size, 0);
    host.packet
        .copy_from_slice(core::slice::from_raw_parts(input, input_size as usize));
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
fn named_severity_levels_preserve_numeric_values() {
    for (value, severity) in [
        MavSeverity::EMERGENCY,
        MavSeverity::ALERT,
        MavSeverity::CRITICAL,
        MavSeverity::ERROR,
        MavSeverity::WARNING,
        MavSeverity::NOTICE,
        MavSeverity::INFO,
        MavSeverity::DEBUG,
    ]
    .into_iter()
    .enumerate()
    {
        assert_eq!(severity, MavSeverity(value as i32));
    }
    assert_eq!(MavSeverity(99).0, 99);
}

#[test]
fn formatted_text_matches_send_text_without_host_allocation() {
    let mut host = Host {
        calls: 0,
        status: api::STATUS_OK,
        packet: [0; 260],
    };
    context(&mut host)
        .gcs()
        .send_text_fmt(MavSeverity::INFO, format_args!("Tick {:.1} {}", 12.25, "é"))
        .unwrap();
    let formatted = host.packet;
    context(&mut host)
        .gcs()
        .send_text(MavSeverity(6), "Tick 12.2 é")
        .unwrap();
    assert_eq!(host.packet, formatted);
    assert_eq!(&formatted[..4], &6i32.to_le_bytes());
    assert_eq!(host.calls, 2);

    context(&mut host)
        .gcs()
        .send_text_fmt(MavSeverity(7), format_args!(""))
        .unwrap();
    assert!(host.packet[4..].iter().all(|byte| *byte == 0));
}

#[test]
fn formatted_text_boundaries_and_invalid_messages() {
    let mut host = Host {
        calls: 0,
        status: api::STATUS_OK,
        packet: [0; 260],
    };
    let full = core::str::from_utf8(&[b'x'; 255]).unwrap();
    context(&mut host)
        .gcs()
        .send_text_fmt(MavSeverity(6), format_args!("{}", full))
        .unwrap();
    assert_eq!(&host.packet[4..259], full.as_bytes());
    assert_eq!(host.packet[259], 0);
    let prefix = core::str::from_utf8(&[b'x'; 253]).unwrap();
    context(&mut host)
        .gcs()
        .send_text_fmt(MavSeverity(6), format_args!("{}é", prefix))
        .unwrap();
    assert_eq!(&host.packet[257..259], "é".as_bytes());
    assert_eq!(host.calls, 2);

    assert_eq!(
        context(&mut host)
            .gcs()
            .send_text_fmt(MavSeverity(6), format_args!("{}x", full),),
        Err(Error::Invalid)
    );
    assert_eq!(
        context(&mut host)
            .gcs()
            .send_text_fmt(MavSeverity(6), format_args!("{}é", full),),
        Err(Error::Invalid)
    );
    assert_eq!(
        context(&mut host)
            .gcs()
            .send_text_fmt(MavSeverity(6), format_args!("{}", "NUL\0text"),),
        Err(Error::Invalid)
    );
    assert_eq!(host.calls, 2);
}

struct Fails;

impl fmt::Display for Fails {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("partial")?;
        Err(fmt::Error)
    }
}

#[test]
fn formatted_text_propagates_errors_without_sending_partial_text() {
    let mut host = Host {
        calls: 0,
        status: api::STATUS_STOPPED,
        packet: [0; 260],
    };
    assert_eq!(
        context(&mut host)
            .gcs()
            .send_text_fmt(MavSeverity(6), format_args!("{}", Fails),),
        Err(Error::Invalid)
    );
    assert_eq!(host.calls, 0);

    for (status, expected) in [
        (api::STATUS_STOPPED, Error::Stopped),
        (api::STATUS_UNAVAILABLE, Error::Unavailable),
        (api::STATUS_BUSY, Error::Busy),
        (api::STATUS_HOST_ERROR, Error::Host),
    ] {
        host.status = status;
        assert_eq!(
            context(&mut host)
                .gcs()
                .send_text_fmt(MavSeverity(6), format_args!("Tick {:.1}", 1.0),),
            Err(expected)
        );
    }
    assert_eq!(host.calls, 4);
}

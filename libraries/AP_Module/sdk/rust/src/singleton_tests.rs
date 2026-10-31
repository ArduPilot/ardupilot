// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error};
use core::{ffi::c_void, marker::PhantomData};

struct Host {
    presence: u8,
    boolean: u8,
    status: i32,
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
        (7, 4) => {
            assert_eq!(input, &[0xe8, 7, 255, 31, 23, 59, 58]);
            output.copy_from_slice(&u32::MAX.to_le_bytes());
        }
        (4, 9) => {
            assert_eq!(input, &[255; 4]);
            output.copy_from_slice(&[host.presence, 0xe8, 7, 2, 29, 12, 34, 56, 4]);
        }
        (1, 14) => {
            assert_eq!(input, &[2]);
            output[0] = host.presence;
            for part in output[1..13].chunks_exact_mut(4) {
                part.copy_from_slice(&(-0.0f32).to_le_bytes());
            }
            output[13] = host.boolean;
        }
        (0, 4) => output.copy_from_slice(&(-0.0f32).to_le_bytes()),
        (2, 0) => assert_eq!(input, &[0x34, 0x12]),
        (1, 4) => {
            assert_eq!(input, &[7]);
            output.copy_from_slice(&i32::MIN.to_le_bytes());
        }
        (4, 2) => {
            assert_eq!(input, &[44, 1, 0, 0]);
            output.copy_from_slice(&[host.presence, 2]);
        }
        (1, 1) => {
            assert_eq!(input, &[5]);
            output[0] = host.boolean;
        }
        (260, 0) => {
            assert_eq!(&input[..4], &6i32.to_le_bytes());
            let message = &input[4..];
            assert!(
                message.iter().all(|b| *b == 0)
                    || (message[..255].iter().all(|b| *b == b'x') && message[255] == 0)
            );
        }
        _ => panic!("unexpected category wire shape"),
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
fn singleton_categories_and_invalid_outputs() {
    let mut host = Host {
        presence: 1,
        boolean: 1,
        status: 0,
    };
    let context = Context {
        api: &API,
        raw: (&mut host as *mut Host).cast(),
        _not_send_sync: PhantomData,
    };
    assert_eq!(
        context.rtc().date_fields_to_clock_s(2024, -1, 31, 23, 59, 58),
        Ok(u32::MAX)
    );
    assert_eq!(
        context.rtc().clock_s_to_date_fields(u32::MAX),
        Ok(Some((2024, 2, 29, 12, 34, 56, 4)))
    );
    let (roll, pitch, yaw, earth) = context.mount().get_angle_target(2).unwrap().unwrap();
    assert_eq!(
        [roll.to_bits(), pitch.to_bits(), yaw.to_bits()],
        [0x80000000; 3]
    );
    assert!(earth);
    assert_eq!(
        context
            .ahrs().get_relative_position_d_home()
            .unwrap()
            .to_bits(),
        0x80000000
    );
    assert_eq!(context.vehicle().nav_script_time_done(0x1234), Ok(()));
    assert_eq!(
        context.mount().get_mode(7),
        Ok(crate::enums::MavMountMode(i32::MIN))
    );
    assert_eq!(
        context.rc().get_aux_cached(crate::enums::RcChannelAuxFunc(300)),
        Ok(Some(2))
    );
    assert_eq!(context.vehicle().set_mode(5), Ok(true)); // fixed ModeReason is not a wire input
    let severity = crate::enums::MavSeverity(6);
    assert_eq!(context.gcs().send_text(severity, ""), Ok(()));
    assert_eq!(
        context.gcs().send_text(severity, core::str::from_utf8(&[b'x'; 255]).unwrap()),
        Ok(())
    );
    assert_eq!(
        context.gcs().send_text(severity, core::str::from_utf8(&[b'x'; 256]).unwrap()),
        Err(Error::Invalid)
    );
    assert_eq!(
        context.gcs().send_text(severity, "embedded\0NUL"),
        Err(Error::Invalid)
    );

    host.boolean = 2;
    assert_eq!(context.mount().get_angle_target(2), Err(Error::Host));
    host.presence = 0;
    // No output, including an invalid bool, is decoded on false.
    assert_eq!(context.mount().get_angle_target(2), Ok(None));
    assert_eq!(context.rtc().clock_s_to_date_fields(u32::MAX), Ok(None));
    host.presence = 2;
    assert_eq!(context.mount().get_angle_target(2), Err(Error::Host));
    assert_eq!(
        context.rtc().clock_s_to_date_fields(u32::MAX),
        Err(Error::Host)
    );

    for (status, error) in [
        (api::STATUS_STOPPED, Error::Stopped),
        (api::STATUS_BUSY, Error::Busy),
        (api::STATUS_UNAVAILABLE, Error::Unavailable),
        (999, Error::Host),
    ] {
        host.status = status;
        assert_eq!(context.mount().get_angle_target(2), Err(error));
        assert_eq!(context.rtc().clock_s_to_date_fields(u32::MAX), Err(error));
        assert_eq!(context.ahrs().get_relative_position_d_home(), Err(error));
        assert_eq!(context.vehicle().nav_script_time_done(0x1234), Err(error));
    }
}

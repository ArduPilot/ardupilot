// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error, Quaternion, Vector3f};
use core::ffi::c_void;
use core::marker::PhantomData;

struct Host {
    status: i32,
    presence: u8,
    calls: u32,
}

unsafe extern "C" fn call(
    context: *mut c_void,
    _: u32,
    _: *const u8,
    ni: u32,
    output: *mut u8,
    no: u32,
) -> i32 {
    let host = &mut *context.cast::<Host>();
    assert_eq!(ni, 0);
    assert!([12, 13, 17].contains(&no));
    let out = core::slice::from_raw_parts_mut(output, no as usize);
    let offset = usize::from(no != 12);
    if offset != 0 {
        out[0] = host.presence;
    }
    for component in out[offset..].chunks_exact_mut(4) {
        component.copy_from_slice(&0x80000000u32.to_le_bytes());
    }
    host.calls += 1;
    host.status
}

#[test]
fn ahrs_values_without_allocation() {
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
        status: 0,
        presence: 1,
        calls: 0,
    };
    let context = Context {
        api: &api,
        raw: (&mut host as *mut Host).cast(),
        _not_send_sync: PhantomData,
    };
    let expected = Vector3f::new(-0.0, -0.0, -0.0).encode();
    assert_eq!(context.ahrs().get_gyro().unwrap().encode(), expected);
    assert_eq!(
        context.ahrs().get_velocity_ned().unwrap().unwrap().encode(),
        expected
    );
    assert_eq!(
        context
            .ahrs()
            .get_relative_position_ned_origin()
            .unwrap()
            .unwrap()
            .encode(),
        expected
    );
    assert_eq!(
        context.ahrs().get_quaternion().unwrap().unwrap().encode(),
        Quaternion::new(-0.0, -0.0, -0.0, -0.0).encode()
    );
    host.presence = 0;
    assert_eq!(context.ahrs().get_velocity_ned(), Ok(None));
    assert_eq!(context.ahrs().get_relative_position_ned_origin(), Ok(None));
    assert_eq!(context.ahrs().get_quaternion(), Ok(None));
    host.presence = 2;
    assert_eq!(context.ahrs().get_velocity_ned(), Err(Error::Host));
    assert_eq!(
        context.ahrs().get_relative_position_ned_origin(),
        Err(Error::Host)
    );
    assert_eq!(context.ahrs().get_quaternion(), Err(Error::Host));
    for (status, error) in [
        (api::STATUS_BUSY, Error::Busy),
        (api::STATUS_UNAVAILABLE, Error::Unavailable),
        (api::STATUS_STOPPED, Error::Stopped),
        (api::STATUS_INVALID, Error::Invalid),
        (api::STATUS_NO_MEMORY, Error::NoMemory),
        (api::STATUS_HOST_ERROR, Error::Host),
        (999, Error::Host),
    ] {
        host.status = status;
        assert_eq!(context.ahrs().get_gyro(), Err(error));
        assert_eq!(context.ahrs().get_velocity_ned(), Err(error));
        assert_eq!(context.ahrs().get_relative_position_ned_origin(), Err(error));
        assert_eq!(context.ahrs().get_quaternion(), Err(error));
    }
    assert_eq!(host.calls, 38);
}

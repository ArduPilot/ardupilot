// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error, Quaternion, Vector3f};
use core::ffi::c_void;
use core::marker::PhantomData;

#[test]
fn component_bits_and_defaults() {
    fn require_traits<T: Copy + Send + Sync>() {}
    require_traits::<Vector3f>();
    require_traits::<Quaternion>();
    assert_eq!(Vector3f::default().encode(), [0; 12]);
    assert_eq!(Quaternion::default(), Quaternion::new(1.0, 0.0, 0.0, 0.0));
    for bits in [
        0, 0x80000000, 1, 0x7f800000, 0xff800000, 0x7fc12345, 0x7f812345,
    ] {
        let value = f32::from_bits(bits);
        let v = Vector3f::new(value, 2.0, -3.0);
        let bytes = v.encode();
        assert_eq!(&bytes[..4], &bits.to_le_bytes());
        let decoded = Vector3f::decode(&bytes);
        assert_eq!(decoded.x.to_bits(), bits);
        assert_eq!(decoded.y, 2.0);
        assert_eq!(decoded.z, -3.0);
        let q = Quaternion::new(4.0, value, -5.0, value);
        let bytes = q.encode();
        assert_eq!(&bytes[4..8], &bits.to_le_bytes());
        assert_eq!(&bytes[12..16], &bits.to_le_bytes());
        let decoded = Quaternion::decode(&bytes);
        assert_eq!(decoded.q1, 4.0);
        assert_eq!(decoded.q2.to_bits(), bits);
        assert_eq!(decoded.q3, -5.0);
        assert_eq!(decoded.q4.to_bits(), bits);
    }
}

unsafe extern "C" fn failed_call(
    context: *mut c_void,
    _: u32,
    _: *const u8,
    _: u32,
    output: *mut u8,
    length: u32,
) -> i32 {
    // Even a host that writes output before failing cannot change Rust values.
    output.write_bytes(0xa5, length as usize);
    *context.cast::<i32>()
}

#[test]
fn no_copyback_on_host_failure() {
    let api = api::HostApi {
        version: api::ABI_VERSION,
        size: core::mem::size_of::<api::HostApi>() as u32,
        fingerprint: api::API_FINGERPRINT,
        // Tests construct the private Context directly. These absent callbacks
        // also make accidental allocation or global-context use fail the test.
        allocate: None,
        deallocate: None,
        time_us: None,
        sleep_ms: None,
        stop_requested: None,
        text: None,
        panic: None,
        binding_call: Some(failed_call),
    };
    for (mut status, error) in [
        (api::STATUS_INVALID, Error::Invalid),
        (api::STATUS_STOPPED, Error::Stopped),
        (api::STATUS_NO_MEMORY, Error::NoMemory),
        (api::STATUS_UNAVAILABLE, Error::Unavailable),
        (api::STATUS_BUSY, Error::Busy),
        (api::STATUS_HOST_ERROR, Error::Host),
        (999, Error::Host),
    ] {
        let context = Context {
            api: &api,
            raw: (&mut status as *mut i32).cast(),
            _not_send_sync: PhantomData,
        };
        let original_v = Vector3f::new(1.0, 2.0, 3.0);
        let original_q = Quaternion::new(0.5, 0.5, 0.5, 0.5);
        let mut v = original_v;
        let mut q = original_q;
        assert_eq!(v.normalize(&context), Err(error));
        assert_eq!(v, original_v);
        assert_eq!(v.rotate_xy(&context, 1.0), Err(error));
        assert_eq!(v, original_v);
        assert_eq!(q.earth_to_body(&context, &mut v), Err(error));
        assert_eq!(v, original_v);
        assert_eq!(q.to_axis_angle(&context, &mut v), Err(error));
        assert_eq!(v, original_v);
        assert_eq!(q.normalize(&context), Err(error));
        assert_eq!(q, original_q);
        assert_eq!(q.from_euler(&context, 0.1, 0.2, 0.3), Err(error));
        assert_eq!(q, original_q);
        assert_eq!(q.from_axis_angle(&context, original_v, 0.1), Err(error));
        assert_eq!(q, original_q);
        assert_eq!(
            q.from_angular_velocity(&context, original_v, 0.1),
            Err(error)
        );
        assert_eq!(q, original_q);
    }
    let mut status = api::STATUS_OK;
    let context = Context {
        api: &api,
        raw: (&mut status as *mut i32).cast(),
        _not_send_sync: PhantomData,
    };
    // The host's 0xa5 byte is not an encoded bool, even with OK status.
    assert_eq!(Vector3f::default().is_zero(&context), Err(Error::Host));
}

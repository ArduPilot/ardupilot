// SPDX-License-Identifier: GPL-3.0-or-later
use crate::{api, Context, Error};
use core::{ffi::c_void, marker::PhantomData};

#[repr(align(4096))]
struct Storage([u8; 8192]);

struct Host {
    size: u32,
    alignment: u32,
    stop_at: u32,
    calls: u32,
    allocations: u32,
    frees: u32,
    fail_allocation: bool,
    misalign: bool,
    presence: u8,
    storage: Storage,
}

impl Host {
    fn new() -> Self {
        Self {
            size: 128,
            alignment: 64,
            stop_at: 0,
            calls: 0,
            allocations: 0,
            frees: 0,
            fail_allocation: false,
            misalign: false,
            presence: 1,
            storage: Storage([0; 8192]),
        }
    }

    fn pointer(&mut self) -> *mut u8 {
        unsafe { self.storage.0.as_mut_ptr().add(usize::from(self.misalign)) }
    }

    fn context(&mut self) -> Context<'_> {
        Context {
            api: &API,
            raw: (self as *mut Self).cast(),
            _not_send_sync: PhantomData,
        }
    }
}

unsafe extern "C" fn allocate(raw: *mut c_void, size: u32, alignment: u32) -> *mut u8 {
    let host = &mut *raw.cast::<Host>();
    assert_eq!((size, alignment), (host.size, host.alignment));
    host.allocations += 1;
    if host.fail_allocation {
        core::ptr::null_mut()
    } else {
        host.pointer()
    }
}

unsafe extern "C" fn deallocate(raw: *mut c_void, pointer: *mut u8) {
    let host = &mut *raw.cast::<Host>();
    assert_eq!(pointer, host.pointer());
    host.frees += 1;
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
    if host.calls == host.stop_at {
        return api::STATUS_STOPPED;
    }
    // Unpatched unit-test imports all contain UINT32_MAX. Distinguish the
    // three wire shapes here; image-loader tests exercise resolved IDs.
    match (ni, no) {
        (0, 8) => {
            assert!(input.is_null());
            let bytes = core::slice::from_raw_parts_mut(output, 8);
            bytes[..4].copy_from_slice(&host.size.to_le_bytes());
            bytes[4..].copy_from_slice(&host.alignment.to_le_bytes());
        }
        (25, 1) | (8, 5) => {
            let bytes = core::slice::from_raw_parts(input, ni as usize);
            let address = u64::from_le_bytes(bytes[..8].try_into().unwrap());
            assert_eq!(address, host.pointer() as usize as u64);
            *output = host.presence;
            if no == 5 {
                core::slice::from_raw_parts_mut(output.add(1), 4)
                    .copy_from_slice(&73.0f32.to_le_bytes());
            }
        }
        _ => panic!("unexpected Parameter wire shape"),
    }
    api::STATUS_OK
}

const API: api::HostApi = api::HostApi {
    version: api::ABI_VERSION,
    size: core::mem::size_of::<api::HostApi>() as u32,
    fingerprint: api::API_FINGERPRINT,
    allocate: Some(allocate),
    deallocate: Some(deallocate),
    time_us: None,
    sleep_ms: None,
    stop_requested: None,
    text: None,
    panic: None,
    binding_call: Some(call),
};

#[test]
fn parameter_host_layout_and_drop() {
    for (size, alignment) in [(8, 4), (16, 8), (128, 64), (8192, 4096)] {
        let mut host = Host::new();
        host.size = size;
        host.alignment = alignment;
        let context = host.context();
        let parameter = context.parameter("VALUE").unwrap().unwrap();
        assert_eq!(parameter.get(), Ok(Some(73.0)));
        drop(parameter);
        assert_eq!((host.calls, host.allocations, host.frees), (3, 1, 1));
    }
}

#[test]
fn parameter_rejects_invalid_layout_without_allocation() {
    for (size, alignment) in [
        (0, 64),
        (128, 0),
        (128, 3),
        (128, 8192),
        (127, 64),
        (16777217, 1),
    ] {
        let mut host = Host::new();
        host.size = size;
        host.alignment = alignment;
        assert!(matches!(
            host.context().parameter("VALUE"),
            Err(Error::Host)
        ));
        assert_eq!((host.calls, host.allocations, host.frees), (1, 0, 0));
    }
    let mut host = Host::new();
    assert!(matches!(host.context().parameter(""), Err(Error::Invalid)));
    assert_eq!((host.calls, host.allocations, host.frees), (0, 0, 0));
}

#[test]
fn parameter_allocation_errors_and_missing_name() {
    let mut host = Host::new();
    host.fail_allocation = true;
    assert!(matches!(
        host.context().parameter("VALUE"),
        Err(Error::NoMemory)
    ));
    assert_eq!((host.calls, host.allocations, host.frees), (1, 1, 0));

    let mut host = Host::new();
    host.misalign = true;
    assert!(matches!(
        host.context().parameter("VALUE"),
        Err(Error::Host)
    ));
    assert_eq!((host.calls, host.allocations, host.frees), (1, 1, 1));

    for presence in [0, 2] {
        let mut host = Host::new();
        host.presence = presence;
        let context = host.context();
        let result = context.parameter("MISSING");
        assert!(if presence == 0 {
            matches!(result, Ok(None))
        } else {
            matches!(result, Err(Error::Host))
        });
        drop(result);
        assert_eq!((host.calls, host.allocations, host.frees), (2, 1, 1));
    }
}

#[test]
fn parameter_stop_at_query_init_and_get() {
    for stop_at in 1..=3 {
        let mut host = Host::new();
        host.stop_at = stop_at;
        let context = host.context();
        if stop_at == 3 {
            let parameter = context.parameter("VALUE").unwrap().unwrap();
            assert_eq!(parameter.get(), Err(Error::Stopped));
            drop(parameter);
        } else {
            assert!(matches!(context.parameter("VALUE"), Err(Error::Stopped)));
        }
        let allocations = u32::from(stop_at != 1);
        assert_eq!(
            (host.calls, host.allocations, host.frees),
            (stop_at, allocations, allocations)
        );
    }
}

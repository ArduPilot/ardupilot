// SPDX-License-Identifier: GPL-3.0-or-later
extern crate std;

use crate::{check_param_table, ParamDef};

crate::param_table! {
    pub struct TestParams, enum T;
    key = 200, prefix = "TST_";

    // @Param: TST_ALT
    // @DisplayName: Test altitude
    // @Description: A float entry
    // @User: Standard
    Alt = 3, "ALT", f32: 2.5;

    // @Param: TST_TIME
    // @DisplayName: Test time
    // @Description: A negative float default
    // @User: Standard
    Time = 1, "TIME", f32: -1;

    // @Param: TST_LONGEST_NM
    // @DisplayName: Longest name
    // @Description: Uses all 16 characters
    // @User: Advanced
    Longest = 7, "LONGEST_NM", f32: 0;
}

const fn def(index: u8, name: &'static str, default: f32) -> ParamDef {
    ParamDef {
        index,
        name,
        default,
    }
}

#[test]
fn macro_keeps_declaration_order_and_checks_at_compile_time() {
    assert_eq!(
        [T::Alt as usize, T::Time as usize, T::Longest as usize],
        [0, 1, 2]
    );
    assert_eq!(TestParams::SIZE, 7);
    assert_eq!(TestParams::DEFS[0].index, 3);
    assert_eq!(TestParams::DEFS[1].default, -1.0);
    assert_eq!(TestParams::DEFS[2].default, 0.0);
    assert!(T::Time.changed(0b010) && !T::Alt.changed(0b010) && T::Longest.changed(0b100));
}

#[test]
#[should_panic(expected = "duplicate parameter index")]
fn duplicate_index_rejected() {
    check_param_table("X_", &[def(1, "A", 0.0), def(1, "B", 0.0)]);
}

#[test]
#[should_panic(expected = "longer than 16")]
fn long_name_rejected() {
    check_param_table("SHOW_", &[def(1, "TWELVE_CHARS", 0.0)]);
}

#[test]
#[should_panic(expected = "index must be 1 to 63")]
fn index_zero_rejected() {
    check_param_table("X_", &[def(0, "A", 0.0)]);
}

#[test]
#[should_panic(expected = "duplicate parameter name")]
fn duplicate_name_rejected() {
    check_param_table("X_", &[def(1, "A", 0.0), def(2, "A", 1.0)]);
}

#[test]
fn last_storage_slot_is_supported() {
    assert_eq!(check_param_table("X_", &[def(63, "A", 0.0)]), 63);
}

crate::param_table! {
    struct Reserved, enum R;
    key = 201, prefix = "TST_";
    size = 63;
    Value = 1, "VALUE", f32: 3.0;
}

#[test]
fn explicit_capacity_preserves_reserved_slots() {
    assert_eq!(Reserved::SIZE, 63);
}

use crate::{api, Context, Error};
use core::{cell::Cell, ffi::c_void, marker::PhantomData};

struct Host {
    now: Cell<u64>,
    busy: Cell<bool>,
    stopped: Cell<bool>,
    reject: Cell<bool>,
    value: Cell<f32>,
    entries: Cell<u32>,
    allocations: Cell<u32>,
}

impl Host {
    fn new() -> Self {
        Self {
            now: Cell::new(0),
            busy: Cell::new(false),
            stopped: Cell::new(false),
            reject: Cell::new(false),
            value: Cell::new(7.0),
            entries: Cell::new(0),
            allocations: Cell::new(0),
        }
    }

    fn context(&self) -> Context<'_> {
        Context {
            api: &API,
            raw: (self as *const Self).cast_mut().cast(),
            _not_send_sync: PhantomData,
        }
    }
}

unsafe extern "C" fn allocate(raw: *mut c_void, size: u32, alignment: u32) -> *mut u8 {
    assert_eq!((size, alignment), (8, 8));
    let host = &*raw.cast::<Host>();
    host.allocations.set(host.allocations.get() + 1);
    std::alloc::alloc_zeroed(std::alloc::Layout::from_size_align(8, 8).unwrap())
}

unsafe extern "C" fn deallocate(raw: *mut c_void, p: *mut u8) {
    let host = &*raw.cast::<Host>();
    host.allocations.set(host.allocations.get() - 1);
    std::alloc::dealloc(p, std::alloc::Layout::from_size_align(8, 8).unwrap());
}

unsafe extern "C" fn now(raw: *mut c_void) -> u64 {
    (&*raw.cast::<Host>()).now.get()
}
unsafe extern "C" fn sleep(raw: *mut c_void, ms: u32) -> i32 {
    let host = &*raw.cast::<Host>();
    host.now.set(host.now.get() + u64::from(ms) * 1000);
    if host.stopped.get() {
        api::STATUS_STOPPED
    } else {
        api::STATUS_OK
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
    let h = &*raw.cast::<Host>();
    if h.busy.replace(false) {
        return api::STATUS_BUSY;
    }
    // Test imports are unpatched. Wire shapes distinguish table, entry and handle calls.
    match (ni, no) {
        (19, 1) => {
            let input = core::slice::from_raw_parts(input, ni as usize);
            assert_eq!(input[0], 201);
            assert_eq!(&input[1..5], b"TST_");
            assert_eq!(input[18], 63);
            *output = u8::from(!h.reject.get());
        }
        (23, 1) => {
            let input = core::slice::from_raw_parts(input, ni as usize);
            assert_eq!(&input[..2], &[201, 1]);
            assert_eq!(&input[2..7], b"VALUE");
            assert_eq!(&input[19..23], &3.0f32.to_le_bytes());
            h.entries.set(h.entries.get() + 1);
            *output = u8::from(!h.reject.get());
        }
        (0, 8) => {
            core::slice::from_raw_parts_mut(output, 8).copy_from_slice(&[8, 0, 0, 0, 8, 0, 0, 0]);
        }
        (25, 1) => {
            let input = core::slice::from_raw_parts(input, ni as usize);
            assert_eq!(&input[8..17], b"TST_VALUE");
            *output = 1;
        }
        (8, 5) => {
            *output = 1;
            core::slice::from_raw_parts_mut(output.add(1), 4)
                .copy_from_slice(&h.value.get().to_le_bytes());
        }
        (12, 1) => {
            let input = core::slice::from_raw_parts(input, ni as usize);
            if !h.reject.get() {
                h.value
                    .set(f32::from_le_bytes(input[8..12].try_into().unwrap()));
            }
            *output = u8::from(!h.reject.get());
        }
        _ => panic!("unexpected table wire shape"),
    }
    api::STATUS_OK
}

const API: api::HostApi = api::HostApi {
    version: api::ABI_VERSION,
    size: core::mem::size_of::<api::HostApi>() as u32,
    fingerprint: api::API_FINGERPRINT,
    allocate: Some(allocate),
    deallocate: Some(deallocate),
    time_us: Some(now),
    sleep_ms: Some(sleep),
    stop_requested: None,
    text: None,
    panic: None,
    binding_call: Some(call),
};

#[test]
fn live_handles_see_external_updates_and_registration_retries() {
    let h = Host::new();
    let c = h.context();
    h.busy.set(true);
    Reserved::register_definitions(&c).unwrap();
    assert_eq!(h.now.get(), 1000);
    assert_eq!(h.entries.get(), 1);
    assert_eq!(h.allocations.get(), 0);
    let p = Reserved::bind(&c, R::Value).unwrap();
    assert_eq!(p.get(), Ok(Some(7.0)));
    h.value.set(11.0);
    assert_eq!(p.get(), Ok(Some(11.0)));
    drop(p);
    assert_eq!(h.allocations.get(), 0);
}

#[test]
fn caching_is_explicit_and_failed_writes_leave_cache_unchanged() {
    let h = Host::new();
    let c = h.context();
    let mut p = Reserved::register(&c).unwrap();
    assert_eq!(p.get(R::Value), 7.0);
    h.value.set(12.0);
    assert_eq!(p.get(R::Value), 7.0);
    assert!(R::Value.changed(p.refresh().unwrap()));
    assert_eq!(p.get(R::Value), 12.0);
    assert_eq!(p.refresh(), Ok(0));
    h.reject.set(true);
    assert_eq!(p.set(R::Value, 13.0), Err(Error::Host));
    assert_eq!(p.set_and_save(R::Value, 13.0), Err(Error::Host));
    assert_eq!(p.get(R::Value), 12.0);
    h.reject.set(false);
    p.set_and_save(R::Value, 14.0).unwrap();
    assert_eq!(h.value.get(), 14.0);
    assert_eq!(p.get(R::Value), 14.0);
    drop(p);
    assert_eq!(h.allocations.get(), 0);
}

#[test]
fn rejected_registration_and_stop_are_propagated() {
    let h = Host::new();
    let c = h.context();
    h.reject.set(true);
    assert_eq!(Reserved::register_definitions(&c), Err(Error::Host));
    h.reject.set(false);
    h.busy.set(true);
    h.stopped.set(true);
    assert_eq!(Reserved::register_definitions(&c), Err(Error::Stopped));
    assert_eq!(h.entries.get(), 0);
}

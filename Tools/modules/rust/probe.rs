// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use core::alloc::{GlobalAlloc, Layout};
use core::panic::PanicInfo;
use core::ptr::{addr_of, addr_of_mut};
use core::sync::atomic::{AtomicPtr, Ordering};

mod memory;

#[repr(C)]
pub struct Host {
    output: extern "C" fn(*const u8, usize),
    sleep: extern "C" fn(u32),
    allocate: extern "C" fn(usize, usize) -> *mut u8,
    deallocate: extern "C" fn(*mut u8, usize, usize),
}

static HOST: AtomicPtr<Host> = AtomicPtr::new(core::ptr::null_mut());

struct Allocator;
unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let host = HOST.load(Ordering::Acquire);
        if host.is_null() { return core::ptr::null_mut(); }
        ((*host).allocate)(layout.size(), layout.align())
    }
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        ((*HOST.load(Ordering::Acquire)).deallocate)(pointer, layout.size(), layout.align());
    }
}

#[global_allocator]
static ALLOCATOR: Allocator = Allocator;

static MESSAGE: &[u8] = b"Native module probe";
static mut COUNTER: u32 = 7;
static mut ZERO: [u32; 8] = [0; 8];
static FUNCTIONS: [fn(u32) -> u32; 2] = [twice, three_times];

#[inline(never)]
fn twice(n: u32) -> u32 { n * 2 }
#[inline(never)]
fn three_times(n: u32) -> u32 { n * 3 }

#[no_mangle]
pub unsafe extern "C" fn start(host: &Host) -> u32 {
    HOST.store(host as *const Host as *mut Host, Ordering::Release);
    let mut values = Vec::<u32>::new();
    if values.try_reserve_exact(64).is_err() {
        HOST.store(core::ptr::null_mut(), Ordering::Release);
        return 0xbad0;
    }
    for i in (0..64).rev() { values.push(core::hint::black_box(i)); }
    values.sort_unstable();
    let correct = values.iter().copied().eq(0..64);
    drop(values);
    if !correct { return 0xbad1; }
    (host.output)(MESSAGE.as_ptr(), MESSAGE.len());
    (host.sleep)(10);
    let n = addr_of!(COUNTER).read_volatile();
    addr_of_mut!(COUNTER).write_volatile(n + 1);
    let zero = addr_of!(ZERO).read_volatile();
    let functions = core::hint::black_box(&FUNCTIONS);
    let value = functions[(n & 1) as usize](n);
    let float = core::hint::black_box(1.25_f32) * core::hint::black_box(8.0_f32);
    // M7 builds exercise hardware double precision; M4 builds use compiler helpers.
    let double = core::hint::black_box(1.25_f64) * core::hint::black_box(8.0_f64);
    if double != 10.0 { return 0xbad2; }
    HOST.store(core::ptr::null_mut(), Ordering::Release);
    value + float as u32 + zero[0]
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop { core::hint::spin_loop(); }
}

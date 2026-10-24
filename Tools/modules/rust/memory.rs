// SPDX-License-Identifier: GPL-3.0-or-later
// Freestanding probe helpers. Volatile accesses prevent recursive lowering to
// the very memory routines being implemented. The production SDK must measure
// and choose its own optimized helpers.

use core::ffi::c_void;

#[no_mangle]
pub unsafe extern "C" fn memcpy(dest: *mut c_void, src: *const c_void, size: usize) -> *mut c_void {
    let dest_bytes = dest.cast::<u8>();
    let src_bytes = src.cast::<u8>();
    for i in 0..size { dest_bytes.add(i).write_volatile(src_bytes.add(i).read_volatile()); }
    dest
}

#[no_mangle]
pub unsafe extern "C" fn memmove(dest: *mut c_void, src: *const c_void, size: usize) -> *mut c_void {
    if (dest as usize) <= src as usize {
        memcpy(dest, src, size)
    } else {
        let dest_bytes = dest.cast::<u8>();
        let src_bytes = src.cast::<u8>();
        for i in (0..size).rev() { dest_bytes.add(i).write_volatile(src_bytes.add(i).read_volatile()); }
        dest
    }
}

#[no_mangle]
pub unsafe extern "C" fn memset(dest: *mut c_void, value: i32, size: usize) -> *mut c_void {
    let dest_bytes = dest.cast::<u8>();
    for i in 0..size { dest_bytes.add(i).write_volatile(value as u8); }
    dest
}

#[no_mangle]
pub unsafe extern "C" fn memcmp(a: *const c_void, b: *const c_void, size: usize) -> i32 {
    let a_bytes = a.cast::<u8>();
    let b_bytes = b.cast::<u8>();
    for i in 0..size {
        let diff = a_bytes.add(i).read_volatile() as i32 - b_bytes.add(i).read_volatile() as i32;
        if diff != 0 { return diff; }
    }
    0
}

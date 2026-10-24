// SPDX-License-Identifier: GPL-3.0-or-later
// Conservative freestanding helpers, as in the relocation probe. Volatile
// accesses prevent recursive lowering to libc. Benchmark before optimizing.

use core::ffi::c_void;

#[no_mangle]
pub unsafe extern "C" fn memcpy(
    destination: *mut c_void,
    source: *const c_void,
    size: usize,
) -> *mut c_void {
    let destination_bytes = destination.cast::<u8>();
    let source_bytes = source.cast::<u8>();
    for index in 0..size {
        destination_bytes
            .add(index)
            .write_volatile(source_bytes.add(index).read_volatile());
    }
    destination
}

#[no_mangle]
pub unsafe extern "C" fn memmove(
    destination: *mut c_void,
    source: *const c_void,
    size: usize,
) -> *mut c_void {
    if destination as usize <= source as usize {
        memcpy(destination, source, size)
    } else {
        let destination_bytes = destination.cast::<u8>();
        let source_bytes = source.cast::<u8>();
        for index in (0..size).rev() {
            destination_bytes
                .add(index)
                .write_volatile(source_bytes.add(index).read_volatile());
        }
        destination
    }
}

#[no_mangle]
pub unsafe extern "C" fn memset(destination: *mut c_void, value: i32, size: usize) -> *mut c_void {
    let destination_bytes = destination.cast::<u8>();
    for index in 0..size {
        destination_bytes.add(index).write_volatile(value as u8);
    }
    destination
}

#[no_mangle]
pub unsafe extern "C" fn memcmp(first: *const c_void, second: *const c_void, size: usize) -> i32 {
    let first_bytes = first.cast::<u8>();
    let second_bytes = second.cast::<u8>();
    for index in 0..size {
        let difference = first_bytes.add(index).read_volatile() as i32
            - second_bytes.add(index).read_volatile() as i32;
        if difference != 0 {
            return difference;
        }
    }
    0
}

#[no_mangle]
pub unsafe extern "C" fn bcmp(first: *const c_void, second: *const c_void, size: usize) -> i32 {
    memcmp(first, second, size)
}

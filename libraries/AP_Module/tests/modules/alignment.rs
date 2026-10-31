// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Result};
use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

ap_rust_sdk::module!(run);

#[repr(align(4096))]
struct Aligned([u8; 4]);

static RX: Aligned = Aligned([1, 2, 3, 4]);
static mut RW: Aligned = Aligned([5, 6, 7, 8]);

fn run(context: &Context<'_>) -> Result {
    let rx = core::hint::black_box(addr_of!(RX));
    let rw = core::hint::black_box(addr_of_mut!(RW));
    assert_eq!(core::hint::black_box(rx as usize) % 4096, 0);
    assert_eq!(core::hint::black_box(rw as usize) % 4096, 0);
    unsafe {
        assert_eq!(read_volatile(addr_of!((*rx).0[1])), 2);
        assert_eq!(read_volatile(addr_of!((*rw).0[1])), 6);
        write_volatile(addr_of_mut!((*rw).0[1]), 42);
        assert_eq!(read_volatile(addr_of!((*rw).0[1])), 42);
    }
    context.text(6, "MATH ok")
}

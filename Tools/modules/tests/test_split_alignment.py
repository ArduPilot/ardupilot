# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Verify split ARM images retain ELF data alignment and byte pointer values."""

import os
import subprocess

from pathlib import Path

import pack
import pytest

HERE = Path(__file__).resolve().parents[1]


def arm_module(tmp_path, alignment, readonly=False, zero_data=False):
    source = tmp_path / 'aligned.rs'
    contents = '''#![no_std]
#![no_main]
use core::ptr::{addr_of_mut, read_volatile};
#[repr(align(ALIGNMENT))]
struct Aligned([u8; 4]);
static mut DATA: Aligned = Aligned([1, 2, 3, 4]);
static mut POINTER: *mut u8 = unsafe { addr_of_mut!(DATA.0).cast::<u8>().add(1) };
#[no_mangle]
pub unsafe extern "C" fn start() -> usize {
    let pointer = read_volatile(addr_of_mut!(POINTER));
    pointer as usize + read_volatile(pointer) as usize
}
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
'''.replace('ALIGNMENT', str(alignment))
    if readonly:
        contents = contents.replace('static mut DATA', 'static DATA')
        contents = contents.replace('addr_of_mut!(DATA.0).cast::<u8>()',
                                    'core::ptr::addr_of!(DATA.0).cast::<u8>().cast_mut()')
    if zero_data:
        contents = contents.replace('Aligned([1, 2, 3, 4])', 'Aligned([0; 4])')
    source.write_text(contents)
    output = tmp_path / 'aligned.elf'
    subprocess.run([os.environ.get('RUSTC', 'rustc'), str(source), '--edition=2021',
                    '--target', 'thumbv7em-none-eabihf', '-C', 'opt-level=s', '-C', 'panic=abort',
                    '-C', 'relocation-model=ropi-rwpi', '-C', f'link-arg=-T{HERE / "module.ld"}',
                    '-C', 'link-arg=--emit-relocs', '-C', 'link-arg=--defsym=MODULE_DATA_ALIGNMENT=4',
                    '-o', str(output)], check=True, capture_output=True)
    manifest = dict(abi_version=1, cpu_profile=1, stack_size=4096, heap_size=32768,
                    fingerprint=bytes(range(1, 17)).hex(), thread_priority=2)
    return pack.pack_elf(output.read_bytes(), manifest)


@pytest.mark.parametrize('alignment', [4, 16, 64, 4096])
def test_rw_alignment_and_odd_byte_pointer(tmp_path, alignment):
    data = arm_module(tmp_path, alignment)
    module = pack.unpack(data)
    assert module['rw_alignment'] == alignment
    if alignment < 4096:
        assert module['rx_size'] < 4096
    pointer_offset, target = next((offset, target) for offset, target in module['fixups']
                                  if offset >= module['rx_size'] and target >= module['rx_size'])
    assert target % alignment == 1
    rw_base = 0x20000000
    loaded = pack.relocate(data, 0x08000000, rw_base)
    pointer = int.from_bytes(loaded[pointer_offset:pointer_offset + 4], 'little')
    assert pointer == rw_base + target - module['rx_size']
    assert pointer & 1
    # ARM RX placement follows ELF alignment, not a hard-coded page boundary.
    loaded = pack.relocate(data, 0x08000000 + module['rx_alignment'], rw_base)
    assert int.from_bytes(loaded[pointer_offset:pointer_offset + 4], 'little') == pointer
    with pytest.raises(ValueError, match='unaligned RW'):
        pack.relocate(data, 0x08000000, rw_base + alignment // 2)


def test_bss_alignment_after_word_data(tmp_path):
    data = arm_module(tmp_path, 64, zero_data=True)
    module = pack.unpack(data)
    assert module['rw_alignment'] == 64
    _, target = next((offset, target) for offset, target in module['fixups']
                     if offset >= module['rx_size'] and target >= module['rx_size'])
    assert (target - module['rx_size']) % 64 == 1


def test_one_past_rx_pointer_at_rw_boundary_rejected(tmp_path):
    source = tmp_path / 'boundary.S'
    source.write_text('''.syntax unified
.eabi_attribute 28, 1
.thumb
.text
.global start
.type start, %function
start: bx lr
.balign 4
.section .rodata
readonly:
.space 4092
.data
.word readonly + 4092
''')
    output = tmp_path / 'boundary.elf'
    subprocess.run(['arm-none-eabi-gcc', '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16',
                    '-nostdlib', f'-Wl,-T{HERE / "module.ld"},--emit-relocs', str(source), '-o', str(output)],
                   check=True, capture_output=True)
    manifest = dict(abi_version=1, cpu_profile=1, stack_size=4096, heap_size=32768,
                    fingerprint=bytes(range(1, 17)).hex(), thread_priority=2)
    with pytest.raises(ValueError, match='ambiguous one-past RX pointer'):
        pack.pack_elf(output.read_bytes(), manifest)


@pytest.mark.parametrize('alignment', [8, 16, 64, 4096])
def test_readonly_alignment_preserved(tmp_path, alignment):
    module = pack.unpack(arm_module(tmp_path, alignment, readonly=True))
    assert module['rx_alignment'] == alignment
    assert module['rw_alignment'] == 4

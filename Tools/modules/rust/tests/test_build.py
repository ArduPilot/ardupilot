# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Unit tests for the Native module builder."""

import pack
import pytest

from elftools.elf.elffile import ELFFile
from rust.build import check_rust_version

import build


@pytest.mark.parametrize('version', [
    'rustc 1.98.1 (example)',
    'rustc 1.99.0 (example)',
    'rustc 1.101.0-nightly (example)',
])
def test_minimum_rust_version_accepts_supported_compilers(version):
    check_rust_version(version)


@pytest.mark.parametrize('version', [
    'rustc 1.98.0 (example)',
    'not rustc',
])
def test_minimum_rust_version_rejects_unsupported_compilers(version):
    with pytest.raises(ValueError):
        check_rust_version(version)


def test_float_remainder_links_without_unwind_dependencies(tmp_path):
    source = tmp_path / 'remainder.rs'
    source.write_text('''#![no_std]
#![no_main]
use ap_rust_sdk::{Context, Result};
ap_rust_sdk::module!(run);
fn run(context: &Context<'_>) -> Result {
    let value = core::hint::black_box(context.time_us() as f32);
    core::hint::black_box(value % 360.0);
    Ok(())
}
''')
    image = build.build(source, tmp_path / 'remainder.apmod', 'x86_64-unknown-linux-gnu', 8192)
    assert image.is_file()
    with image.with_suffix('.elf').open('rb') as binary:
        elf = ELFFile(binary)
        assert all(not symbol.name or symbol['st_shndx'] != 'SHN_UNDEF'
                   for symbol in elf.get_section_by_name('.symtab').iter_symbols())
        assert not any(section.name.startswith(('.eh_frame', '.gcc_except_table')) for section in elf.iter_sections())


@pytest.mark.parametrize('target', list(build.TARGETS))
def test_target_specific_rx_boundary(tmp_path, target):
    source = build.SDK_ROOT / 'examples/simple_loop.rs'
    output = build.build(source, tmp_path / 'loop.apmod', target, 8192)
    module = pack.unpack(output.read_bytes())
    if target == 'thumbv7em-none-eabihf':
        assert module['rx_size'] % 4 == 0
        assert module['rx_size'] % 4096 != 0
        assert len(output.read_bytes()) < 8192
    else:
        assert module['rx_size'] % 4096 == 0

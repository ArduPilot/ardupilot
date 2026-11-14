# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build runtime-free Fortran and run it alongside Rust/C++ in real SITL."""

import io
import math
import os
import re
import subprocess

import apmod_debug
import pack
import pytest

from elftools.elf.elffile import ELFFile
from test_runtime import ROOT
from test_runtime import Vehicle

import build

DEMO = ROOT / 'libraries/AP_Module/examples/simple_loop.f90'
TARGET = 'x86_64-unknown-linux-gnu'


@pytest.mark.parametrize('debug', [False, True])
@pytest.mark.parametrize('target', list(build.TARGETS))
def test_fortran_rebases_without_rust(tmp_path, monkeypatch, debug, target):
    monkeypatch.setenv('RUSTC', '/no/rust/installation')
    if target != TARGET:
        monkeypatch.setenv('FC', os.environ.get('ARM_FC', 'arm-linux-gnueabihf-gfortran'))
    first = build.build(DEMO, tmp_path / 'first.apmod', target, 8192, debug=debug)
    second = build.build(DEMO, tmp_path / 'second.apmod', target, 8192, base=0x24123000, debug=debug)
    assert first.read_bytes() == second.read_bytes()
    image = pack.unpack(first.read_bytes())
    assert image['contract']['stack_size'] == 8192
    assert image['contract']['heap_size'] == 4096
    assert {entry[2] for entry in image['imports']} == {'ahrs.get_gyro', 'gcs.send_text', 'string.format_printf'}
    elf = ELFFile(io.BytesIO(first.with_suffix('.elf').read_bytes()))
    assert all(not symbol.name or symbol['st_shndx'] != 'SHN_UNDEF'
               for symbol in elf.get_section_by_name('.symtab').iter_symbols())
    assert not any(section.name in ('.dynamic', '.plt', '.init_array') for section in elf.iter_sections())
    if target != TARGET:
        for assembly in first.with_suffix('.fortran').glob('*.s'):
            instructions = '\n'.join(line for line in assembly.read_text().splitlines()
                                     if not line.lstrip().startswith(('.', '@')))
            assert not re.search(r'\br9\b', instructions), 'Fortran must leave the SDK static base untouched'
    if debug:
        assert not apmod_debug.ModuleSymbols(first).is_rust


def test_fortran_rejects_wrong_compiler_target(tmp_path, monkeypatch):
    monkeypatch.setenv('FC', 'gfortran')
    with pytest.raises(ValueError, match='incompatible with thumbv7em'):
        build.build(DEMO, tmp_path / 'arm.apmod', 'thumbv7em-none-eabihf', 8192)


def test_arm_fortran_rejects_mutable_statics(tmp_path, monkeypatch):
    monkeypatch.setenv('FC', os.environ.get('ARM_FC', 'arm-linux-gnueabihf-gfortran'))
    source = tmp_path / 'static.f90'
    source.write_text('''function run(context) bind(C, name="ap_module_fortran_run") result(status)
    use iso_c_binding
    implicit none
    type(c_ptr), value :: context
    integer(c_int32_t) :: status
    integer(c_int32_t), save :: counter = 41
    counter = counter + 1
    status = counter
end function
''')
    with pytest.raises(ValueError, match='ARM GOT is unsupported|PC-relative relocation target is not RX'):
        build.build(source, tmp_path / 'static.apmod', 'thumbv7em-none-eabihf', 8192)


@pytest.mark.parametrize('level', ['0', '1', '2', '3', 'z'])
def test_fortran_rejects_unaudited_optimization(tmp_path, level):
    with pytest.raises(ValueError, match='require --opt-level=s'):
        build.build(DEMO, tmp_path / 'unsupported.apmod', TARGET, 8192, opt_level=level)


def test_fortran_rejects_rust_cfg(tmp_path):
    with pytest.raises(ValueError, match='Rust cfg options are not supported for Fortran'):
        build.build(DEMO, tmp_path / 'unsupported.apmod', TARGET, 8192, cfg=('test',))


def test_fortran_missing_compiler(tmp_path, monkeypatch):
    monkeypatch.setenv('FC', '/no/fortran/compiler')
    with pytest.raises(ValueError, match='require GNU Fortran'):
        build.build(DEMO, tmp_path / 'missing.apmod', TARGET, 8192)


@pytest.mark.parametrize('target', list(build.TARGETS))
def test_fortran_io_dependency_fails_build(tmp_path, capfd, target, monkeypatch):
    if target != TARGET:
        monkeypatch.setenv('FC', os.environ.get('ARM_FC', 'arm-linux-gnueabihf-gfortran'))
    source = tmp_path / 'io.f90'
    source.write_text('''function run(context) bind(C, name="ap_module_fortran_run") result(status)
    use iso_c_binding
    implicit none
    type(c_ptr), value :: context
    integer(c_int32_t) :: status
    print *, "Fortran I/O is not part of the demo SDK"
    status = 0
end function
''')
    output = tmp_path / 'io.apmod'
    with pytest.raises(subprocess.CalledProcessError):
        build.build(source, output, target, 8192)
    assert '_gfortran_' in capfd.readouterr().err
    assert not output.exists()


def test_fortran_math_status_and_static_reset_sitl(tmp_path):
    source = tmp_path / 'math.f90'
    source.write_text('''function run(context) bind(C, name="ap_module_fortran_run") result(status)
    use iso_c_binding
    use ap_module
    implicit none
    type(c_ptr), value :: context
    integer(c_int32_t) :: status
    integer(c_int32_t), save :: counter = 41
    real(c_float) :: vector(3)
    status = module_invalid
    if (counter /= 41) return
    counter = counter + 1
    vector = [3.0_c_float, 4.0_c_float, 12.0_c_float]
    status = send_text_floats(context, "Fortran maths %.1f", [sqrt(sum(vector * vector))])
    if (status /= module_ok) return
    status = send_text_floats(context, "%d", [1.0_c_float])
    if (status /= module_invalid) then
        status = -4
        return
    end if
    status = send_text_floats(context, "%f", spread(1.0_c_float, 1, 9))
    if (status /= module_invalid) then
        status = -4
        return
    end if
    status = sleep_ms(context, -1_c_int32_t)
    if (status /= module_invalid) then
        status = -4
        return
    end if
    status = send_text(context, "Fortran checks ok")
    if (status /= module_ok) return
    do while (.not. stop_requested(context))
        status = sleep_ms(context, 1000_c_int32_t)
        if (status == module_stopped) exit
        if (status /= module_ok) return
    end do
    status = module_ok
end function
''')
    image = build.build(source, tmp_path / 'math.apmod', TARGET, 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.text('Fortran maths 13.0')
        vehicle.text('Fortran checks ok')
        for generation in (1, 2):
            mark = len(vehicle.texts)
            vehicle.restart(generation)
            vehicle.text('Fortran maths 13.0', after=mark)
            vehicle.text('Fortran checks ok', after=mark)
        assert not any('returned -' in text or 'panic' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


def test_three_language_sitl_demo(tmp_path, monkeypatch):
    monkeypatch.setenv('AP_MODULE_LIFECYCLE', 'cleanup')
    sources = [DEMO, ROOT / 'libraries/AP_Module/tests/modules/lifecycle.cpp',
               ROOT / 'libraries/AP_Module/tests/modules/lifecycle.rs']
    images = {}
    for index, source in enumerate(sources):
        image = build.build(source, tmp_path / f'{index}.apmod', TARGET, 8192)
        images[image.name] = image.read_bytes()
    vehicle = Vehicle(tmp_path / 'vehicle', images, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        for generation in range(3):
            mark = len(vehicle.texts) if generation else 0
            if generation:
                vehicle.restart(generation, expected=3)
            vehicle.text('Fortran ready', after=mark)
            vehicle.text('Fortran tick ', after=mark)
            vehicle.text('CPP ready', after=mark)
            vehicle.text('Module: ready', after=mark)
        vehicle.pump(0.5)
        samples = [tuple(map(float, match.groups())) for line in vehicle.texts[mark:]
                   if (match := re.search(r'Fortran tick ([0-9.]+) gyro ([0-9.]+)', line))]
        assert len(samples) >= 2, vehicle.diagnostic()
        assert all(math.isfinite(seconds) and math.isfinite(norm) and norm >= 0 for seconds, norm in samples)
        assert samples[-1][0] > samples[0][0]
        assert not any('returned -' in text or 'panic' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()

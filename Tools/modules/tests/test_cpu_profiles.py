# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""CPU requirements must agree across compilation, packaging and inspection."""

import io
import os
import subprocess
import sys

import apmod_info
import pack
import pytest
import targets

from elftools.elf.elffile import ELFFile

import build

ARM = 'thumbv7em-none-eabihf'
X86 = 'x86_64-unknown-linux-gnu'


def test_cpu_selection_defaults_and_explicit_requirements():
    assert targets.select(ARM).id == 1
    assert targets.select(ARM, 'cortex-m4').fpu == 'fpv4-sp-d16'
    assert targets.select(ARM, 'cortex-m7').id == 3
    assert targets.select(ARM, 'cortex-m7').fpu == 'fpv5-d16'
    assert targets.select(X86).id == 2


@pytest.mark.parametrize('machine,profile', [(40, 2), (62, 1), (62, 3), (40, 0), (40, 4), (40, True)])
def test_container_rejects_unknown_or_cross_architecture_profiles(machine, profile):
    with pytest.raises(ValueError, match='invalid CPU profile'):
        pack.validate_contract(machine, 3, profile, 4096, bytes(range(1, 17)), 4096)


@pytest.mark.parametrize('target,cpu', [(ARM, 'x86-64'), (X86, 'cortex-m4'), (X86, 'cortex-m7'), (ARM, 'unknown')])
def test_incompatible_cpus_fail_before_invoking_a_compiler(tmp_path, monkeypatch, target, cpu):
    monkeypatch.setenv('RUSTC', '/no/compiler')
    monkeypatch.setenv('CXX', '/no/compiler')
    monkeypatch.setenv('FC', '/no/compiler')
    for suffix in ('rs', 'cpp', 'f90'):
        with pytest.raises(ValueError, match='incompatible'):
            build.build(tmp_path / ('source.' + suffix), tmp_path / 'bad.apmod', target, 8192, cpu=cpu)


@pytest.mark.parametrize('suffix', ['rs', 'cpp', 'f90'])
@pytest.mark.parametrize('cpu,profile,fp_arch', [('cortex-m4', 1, 6), ('cortex-m7', 3, 8)])
@pytest.mark.parametrize('debug', [False, True])
def test_arm_builds_encode_the_actual_cpu(tmp_path, monkeypatch, suffix, cpu, profile, fp_arch, debug):
    if suffix == 'f90':
        monkeypatch.setenv('FC', os.environ.get('ARM_FC', 'arm-linux-gnueabihf-gfortran'))
    source = build.SDK_ROOT / ('examples/simple_loop.' + suffix)
    image = build.build(source, tmp_path / 'loop.apmod', ARM, 8192, cpu=cpu, debug=debug)
    raw = image.read_bytes()
    contract = pack.unpack(raw)['contract']
    assert contract['cpu_profile'] == profile
    assert contract['thread_priority'] == 2
    elf_data = image.with_suffix('.elf').read_bytes()
    elf = ELFFile(io.BytesIO(elf_data))
    attributes = {attribute.tag: attribute.value
                  for section in elf.get_section_by_name('.ARM.attributes').iter_subsections()
                  for subsection in section.iter_subsubsections()
                  for attribute in subsection.iter_attributes()}
    assert attributes['TAG_CPU_ARCH'] == 13  # Armv7E-M, not Armv8-M.
    assert attributes['TAG_FP_ARCH'] == fp_arch
    assert attributes['TAG_CPU_NAME'].lower() == cpu
    assert 'CPU requirement: ' + cpu in apmod_info.format_module(image, raw, elf_data=elf_data)
    resized = apmod_info.resize_resources(raw, stack_size=16384, heap_size=8192)
    assert pack.unpack(resized)['contract']['cpu_profile'] == profile
    assert pack.pack_elf(elf_data, pack.unpack(resized)['contract']) == resized


def test_cpu_option_reaches_the_cli_builder(tmp_path):
    image = tmp_path / 'm7.apmod'
    subprocess.run([sys.executable, str(build.HERE / 'build.py'), str(build.SDK_ROOT / 'examples/simple_loop.rs'),
                    str(image), '--target', ARM, '--cpu', 'cortex-m7', '--stack-size', '8192', '--heap-size', '4096'],
                   check=True)
    assert pack.unpack(image.read_bytes())['contract']['cpu_profile'] == 3


@pytest.mark.parametrize('suffix,priority', [('rs', 3), ('cpp', 4), ('f90', 5)])
def test_explicit_thread_priority_reaches_every_backend(tmp_path, monkeypatch, suffix, priority):
    if suffix == 'f90':
        monkeypatch.setenv('FC', os.environ.get('ARM_FC', 'arm-linux-gnueabihf-gfortran'))
    source = build.SDK_ROOT / ('examples/simple_loop.' + suffix)
    image = build.build(source, tmp_path / 'priority.apmod', ARM, 8192, thread_priority=priority)
    assert pack.unpack(image.read_bytes())['contract']['thread_priority'] == priority


@pytest.mark.parametrize('priority', [-1, 0, 255, 1 << 64, True, 1.5])
def test_bad_thread_priority_fails_before_compiling(tmp_path, monkeypatch, priority):
    monkeypatch.setenv('RUSTC', '/no/compiler')
    with pytest.raises(ValueError, match='invalid module thread priority'):
        build.build(tmp_path / 'bad.rs', tmp_path / 'bad.apmod', ARM, 8192, thread_priority=priority)

# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Freestanding C++17 backend. Requires Clang and standalone LLD, not Rust."""
import json
import os
import re
import shutil
import subprocess

from pathlib import Path

import pack
import targets

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
SDK = ROOT / 'libraries/AP_Module/sdk/cpp'
GENERATED = ROOT / 'build/modules/libraries/AP_Module'


def compiler_helpers(profile):
    flags = [f'-mcpu={profile.cpu}', '-mthumb', f'-mfpu={profile.fpu}', '-mfloat-abi=hard']
    compiler = os.environ.get('ARM_GCC', 'arm-none-eabi-gcc')
    library = Path(subprocess.check_output([compiler, *flags, '-print-libgcc-file-name'], text=True).strip())
    if not library.is_file() or library.suffix != '.a':
        raise ValueError(f'ARM modules need {profile.cpu} hard-float libgcc.a (set ARM_GCC)')
    return library


def build(source, output, target, stack_size, base=0x10000, cfg=(), debug=False, opt_level='s', heap_size=4096,
          extra_objects=(), cpu=None, thread_priority=None, extra_sources=()):
    profile = targets.select(target, cpu)
    if opt_level != 's':
        raise ValueError('C++ modules currently require --opt-level=s; other relocation output is unaudited')
    compiler = os.environ.get('CXX', 'clang++')
    linker = shutil.which(os.environ.get('LLD', 'ld.lld'))
    if linker is None:
        raise ValueError('C++ module builds require standalone ld.lld (set LLD to its path)')
    for executable, pattern, name in [(compiler, r'clang version (\d+)\.', 'Clang'),
                                      (linker, r'\bLLD (\d+)\.', 'LLD')]:
        version = subprocess.check_output([executable, '--version'], text=True)
        match = re.search(pattern, version)
        if match is None or int(match[1]) < 21:
            raise ValueError(f'C++ modules require {name} 21 or newer; got {version.splitlines()[0]}')
    arm = profile.machine == 40
    contract = json.loads((GENERATED / 'sdk/abi.json').read_text())
    priority = profile.default_thread_priority if thread_priority is None else thread_priority
    contract.update(cpu_profile=profile.id, stack_size=stack_size, heap_size=heap_size, thread_priority=priority)
    pack.validate_contract(profile.machine, contract['abi_version'], profile.id,
                           stack_size, bytes.fromhex(contract['fingerprint']), heap_size, priority)
    output.parent.mkdir(parents=True, exist_ok=True)
    common = [compiler, '--target=arm-none-eabi' if arm else '--target=x86_64-unknown-linux-gnu',
              '-std=c++17', '-Os', '-ffreestanding', '-fno-builtin', '-fno-exceptions', '-fno-rtti',
              '-fno-threadsafe-statics', '-fno-use-cxa-atexit', '-ffunction-sections', '-fdata-sections',
              '-fno-unwind-tables', '-fno-asynchronous-unwind-tables',
              '-I' + str(SDK), '-I' + str(GENERATED), '-I' + str(GENERATED / 'sdk/cpp')]
    if arm:
        common += [f'-mcpu={profile.cpu}', '-mthumb', f'-mfpu={profile.fpu}', '-mfloat-abi=hard',
                   '-fropi', '-frwpi', '-fallow-unsupported']
    else:
        common += ['-fPIC', '-mno-red-zone']
    if debug:
        common += ['-g', '-fno-omit-frame-pointer']
    objects = []
    for index, path in enumerate((source, SDK / 'runtime.cpp', GENERATED / 'sdk/cpp/imports.cpp', *extra_sources)):
        obj = output.with_suffix(f'.{index}.o')
        subprocess.run(common + ['-c', str(path), '-o', str(obj)], check=True)
        objects.append(str(obj))
    elf = output.with_suffix('.elf')
    command = [linker, '-T', str(HERE / 'module.ld'), '--emit-relocs', '--gc-sections',
               f'--defsym=MODULE_BASE={base}', '-o', str(elf)]
    if arm:
        command += ['--defsym=MODULE_DATA_ALIGNMENT=4']
    helpers = [str(compiler_helpers(profile))] if arm else []
    subprocess.run(command + objects + [str(path) for path in extra_objects] + helpers, check=True)
    output.write_bytes(pack.pack_elf(elf.read_bytes(), contract))
    return output

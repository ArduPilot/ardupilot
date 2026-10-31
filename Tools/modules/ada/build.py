# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Experimental, runtime-free GNAT LLVM backend; not GCC GNAT or a full Ada runtime."""

import os
import re
import shutil
import subprocess

from pathlib import Path

import targets

from cpp import build as cpp

SDK = Path(__file__).resolve().parents[3] / 'libraries/AP_Module/sdk/ada'
GENERATED = cpp.GENERATED / 'sdk/ada'


def build(source, output, target, stack_size, base=0x10000, cfg=(), debug=False, opt_level='s', heap_size=4096, cpu=None,
          thread_priority=None):
    profile = targets.select(target, cpu)
    source = source.resolve()
    if opt_level != 's':
        raise ValueError('Ada modules currently require --opt-level=s')
    compiler = shutil.which(os.environ.get('ADA', 'llvm-gnat1'))
    if compiler is None:
        raise ValueError('Ada modules require GNAT LLVM (set ADA to llvm-gnat1, not GCC GNAT)')
    backend = shutil.which(os.environ.get('LLVM_LLC', 'llc'))
    if backend is None:
        raise ValueError('Ada modules require LLVM llc 21 or newer (set LLVM_LLC)')
    version = subprocess.check_output([backend, '--version'], text=True)
    match = re.search(r'LLVM version (\d+)\.', version)
    if match is None or int(match[1]) < 21:
        raise ValueError('Ada modules require LLVM llc 21 or newer')
    output.parent.mkdir(parents=True, exist_ok=True)
    directory = output.resolve().with_suffix('.ada')
    directory.mkdir(exist_ok=True)
    arm = profile.machine == 40
    common = [compiler, '-c', '-S', '-emit-llvm', '-Os', '-gnat2022', '-gnata', '-gnato',
              '-ffunction-sections', '-fdata-sections', f'--target={target}', f'-mcpu={profile.cpu}',
              '-gnatec=' + str(SDK / 'gnat.adc'), '-I' + str(SDK), '-I' + str(GENERATED),
              '-I' + str(source.resolve().parent)]
    if not arm:
        common += ['-mno-red-zone']
    if debug:
        common += ['-g']
    objects = []
    for index, path in enumerate((SDK / 'ap_module.adb', GENERATED / 'ap_module-types.ads',
                                  GENERATED / 'ap_module-bindings.adb', source)):
        unit_dir = directory / str(index)
        unit_dir.mkdir(exist_ok=True)
        ir = unit_dir / (path.stem + '.ll')
        obj = unit_dir / (path.stem + '.o')
        # The frontend (unlike llvm-gcc) writes unit-named IR and ALI files
        # in its working directory. Keep both out of the source tree.
        subprocess.run(common + [str(path.resolve())], cwd=unit_dir, check=True)
        # GNAT LLVM can emit IR but does not expose ROPI/RWPI switches. Let
        # llc select the same addressing model as Rust and Clang C++ instead.
        # Checks call our non-returning panic helpers; no Ada runtime is linked.
        # Select DWARF metadata so module.ld discards it like Rust/C++ unwind
        # tables, without an ARM EHABI personality dependency or disabling checks.
        command = [backend, '-filetype=obj', '-O2', '-function-sections', '-data-sections',
                   '-relocation-model=ropi-rwpi' if arm else '-relocation-model=pic',
                   '-exception-model=dwarf', f'-mcpu={profile.cpu}']
        if debug:
            # module.ld discards .eh_frame. Keep debugger-only CFI for Ada
            # frames in the companion ELF, as the Rust backend does.
            command += ['--force-dwarf-frame-section']
        if arm:
            command += ['-float-abi=hard']
        subprocess.run(command + [str(ir), '-o', str(obj)], check=True)
        objects.append(obj)
    if arm:
        # Cortex-M4 needs compiler helpers for double arithmetic, just as the
        # Fortran backend does. This is libgcc, not the GNAT runtime; only
        # referenced helpers are extracted and the packer validates them too.
        flags = [f'-mcpu={profile.cpu}', '-mthumb', f'-mfpu={profile.fpu}', '-mfloat-abi=hard']
        gcc = os.environ.get('ARM_GCC', 'arm-none-eabi-gcc')
        library = Path(subprocess.check_output([gcc, *flags, '-print-libgcc-file-name'], text=True).strip())
        if not library.is_file() or library.suffix != '.a':
            raise ValueError('ARM Ada needs the matching hard-float libgcc.a (set ARM_GCC)')
        objects.append(library)
    # No binder or libgnat: the configuration rejects elaboration code and
    # unsupported runtime features. Missing dependencies are link-time errors.
    return cpp.build(SDK / 'bridge.cpp', output, target, stack_size, base=base, debug=debug,
                     opt_level=opt_level, heap_size=heap_size, extra_objects=objects, cpu=profile.cpu,
                     thread_priority=thread_priority, extra_sources=(GENERATED / 'bridge.cpp',))

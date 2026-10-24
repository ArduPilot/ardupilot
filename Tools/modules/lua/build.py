#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Package ArduPilot's C Lua interpreter as a freestanding native module."""

import argparse
import json
import os
import subprocess
import sys

from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import targets  # noqa: E402

from cpp import build as cpp  # noqa: E402

import build as modules  # noqa: E402

SOURCE = modules.SDK_ROOT / 'applets/lua'
LUA = modules.SDK_ROOT.parent / 'AP_Scripting/lua/src'
CORE = ('lapi lcode lctype ldebug ldo ldump lfunc lgc llex lmem lobject lopcodes '
        'lparser lstate lstring ltable ltm lundump lvm lzio lauxlib lbaselib lmathlib lstrlib ltablib').split()


def build(output, target, stack_size=24576, heap_size=131072, cpu=None, base=0x10000, debug=False,
          script_dir=None, instructions=10000):
    profile = targets.select(target, cpu or ('cortex-m7' if target == 'thumbv7em-none-eabihf' else None))
    arm = profile.machine == 40
    if arm and profile.cpu != 'cortex-m7':
        raise ValueError('the Lua module currently requires the Cortex-M7 double-precision FPU profile')
    script_dir = script_dir if script_dir is not None else ('/APM/scripts' if arm else './scripts')
    if not script_dir or '\0' in script_dir or len(script_dir.encode()) > 190:
        raise ValueError('script directory must contain 1..190 non-NUL bytes')
    if not 1000 <= instructions <= 1000000:
        raise ValueError('instruction budget must be between 1000 and 1000000')
    modules.generate_bindings()
    output = Path(output)
    work = output.with_suffix('.lua-build')
    work.mkdir(parents=True, exist_ok=True)
    # Private headers keep firmware dependencies out of the interpreter build.
    common = ['--target=arm-none-eabi' if arm else '--target=x86_64-unknown-linux-gnu',
              '-Os', '-ffreestanding', '-fno-builtin', '-ffunction-sections', '-fdata-sections',
              '-fno-unwind-tables', '-fno-asynchronous-unwind-tables',
              '-DLUA_32BITS=1', '-DARDUPILOT_BUILD=1', '-DLUAI_MAXCCALLS=32', '-DMAXCCALLS=200',
              '-DLUA_MODULE_FINALIZER_HOOKS=1',
              '-I' + str(SOURCE / 'include'), '-I' + str(LUA)]
    if arm:
        common += [f'-mcpu={profile.cpu}', '-mthumb', f'-mfpu={profile.fpu}', '-mfloat-abi=hard',
                   '-fropi', '-frwpi', '-fallow-unsupported']
    else:
        common += ['-fPIC', '-mno-red-zone']
    if debug:
        common += ['-g', '-fno-omit-frame-pointer']
    objects = []
    for path in [LUA / (name + '.c') for name in CORE] + [SOURCE / 'libc.c', SOURCE / 'setjmp.S']:
        obj = work / (path.stem + '.o')
        # The parser needs more nesting than C callbacks such as recursive
        # gsub, whose per-level buffers consume substantially more native stack.
        limits = ['-ULUAI_MAXCCALLS', '-DLUAI_MAXCCALLS=12'] if path.name == 'ldo.c' else []
        subprocess.run([os.environ.get('CC', 'clang'), *common, *limits, '-c', str(path), '-o', str(obj)], check=True)
        objects.append(obj)
    # The C++ backend owns entry, allocation, import generation and packaging.
    # Its entry source includes these private headers via the generated driver.
    driver = work / 'entry.cpp'
    driver.write_text('#define LUA_MODULE_DIRECTORY ' + json.dumps(script_dir) + '\n'
                      '#define LUA_MODULE_INSTRUCTIONS ' + str(instructions) + '\n'
                      '#include ' + json.dumps(str(SOURCE / 'main.cpp')) + '\n')
    # Compile the bridge separately, so private C headers never affect SDK runtime.cpp.
    bridge = work / 'bridge.o'
    subprocess.run([os.environ.get('CXX', 'clang++'), *common, '-std=c++17', '-fno-exceptions', '-fno-rtti',
                    '-fno-threadsafe-statics', '-fno-use-cxa-atexit',
                    '-I' + str(modules.SDK_ROOT / 'sdk/cpp'), '-I' + str(modules.GENERATED_ROOT),
                    '-I' + str(modules.GENERATED_ROOT / 'sdk/cpp'),
                    '-c', str(driver), '-o', str(bridge)], check=True)
    objects.append(bridge)
    entry = work / 'module.cpp'
    entry.write_text('#include <ap_module.hpp>\n'
                     'extern ap_module::Result<void> lua_module_run(ap_module::Context &);\n'
                     'AP_MODULE(lua_module_run)\n')
    return cpp.build(entry, output, target, stack_size, base=base, heap_size=heap_size,
                     cpu=profile.cpu, debug=debug, extra_objects=objects)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--target', required=True, choices=targets.TARGETS)
    parser.add_argument('--cpu', choices=('cortex-m7', 'x86-64'))
    parser.add_argument('--stack-size', type=int, default=24576)
    parser.add_argument('--heap-size', type=int, default=131072)
    parser.add_argument('--script-dir')
    parser.add_argument('--instructions', type=int, default=10000)
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()
    build(**vars(args))


if __name__ == '__main__':
    main()

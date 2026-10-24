#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build the runtime-free aerobatics applet using the shared Fortran backend."""

import argparse
import shutil
import sys

from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
sys.path.insert(0, str(ROOT / 'Tools/modules'))

from fortran import build as backend  # noqa: E402

import build as module_build  # noqa: E402

SOURCES = ('support', 'path', 'schedule', 'controller', 'parameters', 'mission', 'logging', 'main')


def build(output, target, cpu=None, stack_size=16384, heap_size=32768, debug=False, entry=None):
    output = Path(output).resolve()
    work = output.with_suffix('.aerobatics')
    work.mkdir(parents=True, exist_ok=True)
    source = work / 'aerobatics.f90'
    parts = [(HERE / f'{name}.f90').read_text() for name in SOURCES[:-1]]
    parts.append((Path(entry) if entry is not None else HERE / 'main.f90').read_text())
    source.write_text('\n'.join(parts))
    sdk = work / 'sdk'
    sdk.mkdir(exist_ok=True)
    original_sdk = backend.SDK
    shutil.copyfile(original_sdk / 'ap_module.f90', sdk / 'ap_module.f90')
    bridge = '#include "' + str(original_sdk / 'bridge.cpp') + '"\n'
    bridge += '#include "' + str(HERE / 'memory.cpp') + '"\n'
    (sdk / 'bridge.cpp').write_text(bridge)
    backend.SDK = sdk
    try:
        return module_build.build(source, output, target, stack_size, heap_size=heap_size, cpu=cpu, debug=debug)
    finally:
        backend.SDK = original_sdk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--target', required=True, choices=('x86_64-unknown-linux-gnu', 'thumbv7em-none-eabihf'))
    parser.add_argument('--cpu', choices=('cortex-m4', 'cortex-m7', 'x86-64'))
    parser.add_argument('--stack-size', type=int, default=16384)
    parser.add_argument('--heap-size', type=int, default=32768)
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()
    build(args.output, args.target, args.cpu, args.stack_size, args.heap_size, args.debug)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build the H743 ModuleRuntime example with a native module in ROMFS."""

import argparse
import os
import subprocess

from pathlib import Path

import build

ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT / 'build/modules/romfs_runtime/romfs.apmod'
HWDEF = ROOT / 'Tools/modules/romfs-runtime.hwdef'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--board', default='MatekH743', help='ChibiOS H7 board supported by the Renode fixture')
    parser.add_argument('--out', type=Path, default=ROOT / 'build/romfs-runtime', help='separate waf build directory')
    parser.add_argument('--language', choices=('rust', 'cpp', 'fortran'), default='rust', help='language of the ROMFS module')
    parser.add_argument('--cpu', choices=('cortex-m4', 'cortex-m7'), default='cortex-m4', help='ARM module CPU requirement')
    args = parser.parse_args()

    suffix = {'rust': 'rs', 'cpp': 'cpp', 'fortran': 'F90'}[args.language]
    source = ROOT / f'Tools/modules/{args.language}/romfs_probe.{suffix}'
    if args.language == 'fortran':
        os.environ['FC'] = os.environ.get('ARM_FC', os.environ.get('FC', 'arm-linux-gnueabihf-gfortran'))
    build.build(source, MODULE,
                'thumbv7em-none-eabihf', 8193, cpu=args.cpu)

    subprocess.run([str(ROOT / 'waf'), 'configure', '--board', args.board,
                    '--out', str(args.out), '--extra-hwdef', str(HWDEF)], cwd=ROOT, check=True)
    subprocess.run([str(ROOT / 'waf'), '--targets', 'examples/ModuleRuntime'], cwd=ROOT, check=True)
    print(args.out / args.board / 'examples/ModuleRuntime')


if __name__ == '__main__':
    main()

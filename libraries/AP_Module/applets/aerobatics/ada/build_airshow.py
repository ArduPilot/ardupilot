#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build the Ada AirShow with its clock adapter using the standard Ada backend."""

import argparse
import sys

from pathlib import Path
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
sys.path.insert(0, str(ROOT / 'Tools/modules'))

from ada import build as ada_build  # noqa: E402

import build as module_build  # noqa: E402


def build(source, output, target, stack_size, **options):
    cpp_build = ada_build.cpp.build

    def with_clock(*args, extra_sources=(), **kwargs):
        return cpp_build(*args, extra_sources=(*extra_sources, HERE / 'clock.cpp'), **kwargs)

    with patch.object(ada_build.cpp, 'build', with_clock):
        return module_build.build(source, output, target, stack_size, **options)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--target', required=True)
    parser.add_argument('--cpu')
    parser.add_argument('--stack-size', type=int, default=16384)
    parser.add_argument('--heap-size', type=int, default=4096)
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()
    build(HERE / 'aerobatics.adb', args.output, args.target, args.stack_size,
          heap_size=args.heap_size, cpu=args.cpu, debug=args.debug)


if __name__ == '__main__':
    main()

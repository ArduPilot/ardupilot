#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Extract all DataFlash logs from a Renode SD card image."""

import argparse
import sys

from pathlib import Path

import fat_image


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path, help='path to sdcard.img')
    parser.add_argument(
        '--output-dir', type=Path, default=Path.cwd(),
        help='directory for extracted BIN logs (default: current directory)')
    args = parser.parse_args(argv)

    image = args.image.expanduser().resolve()
    output_directory = args.output_dir.expanduser().resolve()
    if not image.is_file():
        parser.error('SD image does not exist: %s' % image)
    output_directory.mkdir(parents=True, exist_ok=True)
    try:
        logs = fat_image.extract_files(
            image, '/APM/LOGS', '*.BIN', output_directory)
    except RuntimeError as error:
        print('failed to extract logs: %s' % error, file=sys.stderr)
        return 1
    if not logs:
        print('no BIN logs found in %s' % image, file=sys.stderr)
        return 1
    for log in logs:
        print(log)
    print('extracted %u log%s to %s' % (
        len(logs), '' if len(logs) == 1 else 's', output_directory))
    return 0


if __name__ == '__main__':
    sys.exit(main())

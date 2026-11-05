#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Regenerate the applet CRC-extra catalogue from the firmware dialect header."""
import argparse
from pathlib import Path
import re


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('header', type=Path)
    args = parser.parse_args()
    lines = args.header.read_text().splitlines()
    line = next(line for line in lines if line.startswith('#define MAVLINK_MESSAGE_CRCS '))
    entries = re.findall(r'\{(\d+), (\d+),[^{}]+\}', line)
    if not entries or len({key for key, _ in entries}) != len(entries):
        raise ValueError('invalid MAVLink CRC catalogue')
    text = ('// SPDX-License-Identifier: GPL-3.0-or-later\n'
            '// CRC extras from the repository generated ardupilotmega MAVLink dialect.\n'
            'pub fn extra(id: u32) -> Option<u8> {\n    match id {\n')
    text += ''.join(f'        {key} => Some({extra}),\n' for key, extra in entries)
    text += '        _ => None,\n    }\n}\n'
    Path(__file__).with_name('mavlink_crc.rs').write_text(text)


if __name__ == '__main__':
    main()

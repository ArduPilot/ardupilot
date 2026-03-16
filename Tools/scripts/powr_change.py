#!/usr/bin/env python3

"""
Parses a log file and shows how the power flags changed over time.
Uses BitmaskChange.

AP_FLAKE8_CLEAN

"""

import sys

from bitmask_change import BitmaskChange

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: powr_change.py <logfile>", file=sys.stderr)
        sys.exit(1)
    fields = [
        ("POWR", "Flags"),
        ("POWR", "AccFlags", True),   # optional: not present in all firmware
    ]
    BitmaskChange(sys.argv[1], fields).run()

#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Aggregate per-board size diff JSON files (produced by size_diff_to_json.py)
into a single Markdown summary table written to stdout.

Typical usage (from a GitHub Actions step):

    python3 Tools/scripts/build_tests/global_size_summary.py \\
        --input-dir size-diffs/ >> $GITHUB_STEP_SUMMARY
"""

import json
import os
import sys

from argparse import ArgumentParser

COLUMNS = [
    "AP_Periph",
    "antennatracker",
    "blimp",
    "bootloader",
    "copter",
    "heli",
    "iofirmware",
    "plane",
    "rover",
    "sub",
]

parser = ArgumentParser(description="Generate global size summary table.")
parser.add_argument("--input-dir", required=True, help="Directory with per-board JSON files")
args = parser.parse_args()


def fmt_delta(delta):
    """Format an integer byte delta for the table cell."""
    if delta is None:
        return "N/A"
    if delta == 0:
        return "0"
    if delta > 0:
        return "+%d" % delta
    return str(delta)


rows = []
for fname in sorted(os.listdir(args.input_dir)):
    if not fname.endswith(".json"):
        continue
    fpath = os.path.join(args.input_dir, fname)
    try:
        with open(fpath) as f:
            data = json.load(f)
    except (OSError, ValueError) as exc:
        print("WARNING: could not read %s: %s" % (fpath, exc), file=sys.stderr)
        continue

    board = data.get("board", os.path.splitext(fname)[0])
    binaries = data.get("binaries", {})
    row = [board] + [fmt_delta(binaries.get(col)) for col in COLUMNS]
    rows.append(row)

if not rows:
    print("No size diff data found.", file=sys.stderr)
    sys.exit(0)

header = ["Board"] + COLUMNS
# Column alignment: left for Board, centre for each binary
sep = [":---"] + [":---:" for _ in COLUMNS]

lines = [
    "## Global Size Summary (Total Flash delta in bytes)",
    "",
    "| " + " | ".join(header) + " |",
    "| " + " | ".join(sep) + " |",
]
for row in rows:
    lines.append("| " + " | ".join(str(c) for c in row) + " |")
lines.append("")

print("\n".join(lines))

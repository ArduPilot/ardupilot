#!/usr/bin/env python3

'''
This script intend to provide a pretty size diff between two binaries.
It also optionally emits a JSON file suitable for aggregation into a
cross-board summary table by global_size_summary.py.

AP_FLAKE8_CLEAN
'''

import json
import os
import shutil
import subprocess
import sys
import tempfile

from argparse import ArgumentParser

from tabulate import tabulate

# Map lowercased binary filename (no extension) to the column name used in
# the global summary table.  Entries not in this map are silently ignored.
BINARY_TO_COLUMN = {
    "arducopter": "copter",
    "arducopter-heli": "heli",
    "arduplane": "plane",
    "ardurover": "rover",
    "ardusub": "sub",
    "antennatracker": "antennatracker",
    "blimp": "blimp",
    "iofirmware": "iofirmware",
    "ap_periph": "AP_Periph",
    "ap_bootloader": "bootloader",
}

parser = ArgumentParser(description="Print binary size difference with master.")
parser.add_argument("-m", "--master", dest='master', type=str, help="Master Binaries Path", required=True)
parser.add_argument("-s", "--second", dest='second', type=str, help="Second Binaries Path", required=True)
parser.add_argument("--board", dest='board', type=str, default=None,
                    help="Board name (required when --json-output is used)")
parser.add_argument("--json-output", dest='json_output', type=str, default=None,
                    help="Path to write per-board size diff JSON for global_size_summary.py")
parser.add_argument("--toolchain", dest='toolchain', type=str, default="arm-none-eabi",
                    help="Toolchain prefix for strip (default: arm-none-eabi)")

args = parser.parse_args()

if args.json_output and not args.board:
    print("ERROR: --board is required when --json-output is specified", file=sys.stderr)
    sys.exit(1)


def _raw_equal(file1, file2):
    return open(file1, "rb").read() == open(file2, "rb").read()


def _stripped_equal(file1, file2, toolchain):
    """Strip debug symbols from both ELFs into temp files and compare.

    Mirrors size_compare_branches.py:create_stripped_elf — symbol renames
    don't count as real firmware changes.
    """
    strip = "strip" if toolchain is None else f"{toolchain}-strip"
    try:
        with tempfile.NamedTemporaryFile(suffix="-stripped", delete=False) as t1, \
             tempfile.NamedTemporaryFile(suffix="-stripped", delete=False) as t2:
            tmp1, tmp2 = t1.name, t2.name
        shutil.copy(file1, tmp1)
        shutil.copy(file2, tmp2)
        subprocess.run([strip, tmp1], check=True, capture_output=True)
        subprocess.run([strip, tmp2], check=True, capture_output=True)
        return _raw_equal(tmp1, tmp2)
    except (OSError, subprocess.CalledProcessError):
        return False
    finally:
        for f in (tmp1, tmp2):
            try:
                os.unlink(f)
            except OSError:
                pass


def binaries_are_identical(dir1, name, dir2, toolchain="arm-none-eabi"):
    """Return True if the named binary is identical in both dirs.

    Logic mirrors size_compare_branches.py:compare_results_sizes:
      1. prefer .bin (byte-for-byte compare)
      2. fall back to .elf; if they differ, strip and re-compare
         so debug-symbol-only changes don't count as real differences.
    """
    for ext in (".bin", ".elf"):
        p1 = os.path.join(dir1, name + ext)
        p2 = os.path.join(dir2, name + ext)
        if not (os.path.exists(p1) and os.path.exists(p2)):
            continue
        if _raw_equal(p1, p2):
            return True
        if ext == ".elf":
            return _stripped_equal(p1, p2, toolchain)
        return False
    return False


def sizes_for_file(filepath):
    """Get binary size information with size."""
    print("Get binary size of %s" % filepath)
    cmd = "size %s" % (filepath,)
    stuff = os.popen(cmd).read()
    lines = stuff.splitlines()[1:]
    size_list = []
    for line in lines:
        row = line.strip().split()
        size_list.append(dict(
            text=int(row[0]),
            data=int(row[1]),
            bss=int(row[2]),
            total=int(row[3]),
        ))

    # Get the size of .crash_log to remove it from .bss reporting
    cmd = "size -A %s" % (filepath,)
    output = os.popen(cmd).read()
    lines = output.splitlines()[1:]
    for line in lines:
        if ".crash_log" in line:
            row = line.strip().split()
            size_list[0]["crash_log"] = int(row[1])
            break

    # check if crash_log wasn't found and set to 0B if not found
    # FIX ME : so it doesn't report Flash_Free 0B for non-ARM boards
    if size_list[0].get("crash_log") is None:
        size_list[0]["crash_log"] = 0

    return size_list


def print_table(summary_data_list_second, summary_data_list_master):
    """Print the binaries size diff on a table and optionally emit a JSON diff file."""
    print_data = []
    json_binaries = {}
    print("")
    # print(summary_data_list_second)
    # print(summary_data_list_master)
    for name in summary_data_list_second[0]:
        for master_name in summary_data_list_master[0]:
            if name == master_name:
                col_data = [name]
                for key in ["text", "data", "bss", "total"]:
                    bvalue = summary_data_list_second[0][name].get(key)
                    mvalue = summary_data_list_master[0][name].get(key)

                    # BSS: remove the portion occupied by crash_log as the command `size binary.elf`
                    # reports BSS with crash_log included
                    if key == "bss":
                        mvalue = (summary_data_list_master[0][name].get("bss") -
                                  summary_data_list_master[0][name].get("crash_log"))
                        bvalue = (summary_data_list_second[0][name].get("bss") -
                                  summary_data_list_second[0][name].get("crash_log"))

                    # Total Flash Cost = Data + Text
                    if key == "total":
                        mvalue = (summary_data_list_master[0][name].get("text") +
                                  summary_data_list_master[0][name].get("data"))
                        bvalue = (summary_data_list_second[0][name].get("text") +
                                  summary_data_list_second[0][name].get("data"))
                    diff = (bvalue - mvalue) * 100.0 / mvalue
                    signum = "+" if diff > 0.0 else ""
                    print_diff = str(bvalue - mvalue)
                    print_diff += " (" + signum + "%0.4f%%" % diff + ")"
                    col_data.append(print_diff)

                    # Collect total-flash delta for JSON output
                    if key == "total":
                        col = BINARY_TO_COLUMN.get(name.lower())
                        if col is not None:
                            identical = binaries_are_identical(
                                args.master, name, args.second, args.toolchain,
                            )
                            json_binaries[col] = {"delta": bvalue - mvalue, "identical": identical}

                # Append free flash space which is equivalent to crash_log's size
                col_data.append(str(summary_data_list_second[0][name].get("crash_log")))
                print_data.append(col_data)
    print(tabulate(print_data, headers=["Binary Name", "Text [B]", "Data [B]", "BSS (B)",
                                        "Total Flash Change [B] (%)", "Flash Free After PR (B)"]))

    # Write per-board JSON diff if requested
    if args.json_output:
        output = {"board": args.board, "binaries": json_binaries}
        with open(args.json_output, "w") as f:
            json.dump(output, f, indent=2)
        print("Saved size diff JSON to %s" % args.json_output)


def extract_binaries_size(path):
    """Search and extract binary size for each binary in the given path."""
    print("Extracting binaries size on %s" % path)
    binaries_list = []
    for file in os.listdir(args.master):
        fileNoExt = os.path.splitext(file)[0]
        binaries_list.append(fileNoExt)
    binaries_list = list(dict.fromkeys(binaries_list))
    if len(binaries_list) == 0:
        print("Failed to get binaries")

    size_dict = None
    for binaries in binaries_list:
        binary_path = os.path.join(path, binaries)
        parsed = sizes_for_file(binary_path)
        if size_dict is None:
            size_dict = [{binaries.lower(): parsed[0]}]
        else:
            size_dict[0].update({binaries.lower(): parsed[0]})
    print("Success !")
    return size_dict


master_dict = extract_binaries_size(args.master)
second_dict = extract_binaries_size(args.second)

print_table(second_dict, master_dict)

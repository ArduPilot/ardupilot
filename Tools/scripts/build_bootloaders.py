#!/usr/bin/env python3

# flake8: noqa

"""
script to build all of our bootloaders using AP_Bootloader and put the resulting binaries in Tools/bootloaders
"""

import os
import shutil
import subprocess
import sys
import fnmatch
import re

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='This Program is used to build ArduPilot bootloaders for boards.')
parser.add_argument("--signing-key", type=str, action='append', help="signing key for secure bootloader (can be used multiple times)")
parser.add_argument("--omit-ardupilot-keys", action='store_true', default=False, help="omit ArduPilot signing keys")
parser.add_argument("--debug", action='store_true', default=False, help="configure as debug variant")
parser.add_argument("--debug-symbols", "-g", action='store_true', default=False, help="add debug symbols to build")
parser.add_argument("--periph-only", action='store_true', default=False, help="only build AP_Periph boards")
parser.add_argument("pattern", type=str, default='*', help="board wildcard pattern", nargs='?')
args = parser.parse_args()

os.environ['PYTHONUNBUFFERED'] = '1'

failed_boards = set()

# Each HAL keeps its board hwdefs in its own tree. Order matters only if a board
# name appears under both, which it must not.
HWDEF_ROOTS = (
    ('ChibiOS', os.path.join('libraries', 'AP_HAL_ChibiOS', 'hwdef')),
    ('Zephyr', os.path.join('libraries', 'AP_HAL_Zephyr', 'hwdef')),
)


def find_hwdef_bl(board):
    """Return (hal, path) for the board's hwdef-bl.dat, or (None, None)."""
    for hal, root in HWDEF_ROOTS:
        hwdef = os.path.join(root, board, 'hwdef-bl.dat')
        if os.path.exists(hwdef):
            return hal, hwdef
    return None, None


def board_hal(board):
    """Which HAL declares this board, or None if no HAL does."""
    return find_hwdef_bl(board)[0]


def has_hwdef_bl(board):
    """Return True if any HAL has hwdef/<board>/hwdef-bl.dat"""
    return find_hwdef_bl(board)[1] is not None


def read_hwdef(filepath):
    '''read a hwdef file recursively'''
    fh = open(filepath)
    ret = []
    text = fh.readlines()
    for line in text:
        m = re.match(r"^\s*include\s+(.+)\s*$", line)
        if m is not None:
            ret += read_hwdef(os.path.join(os.path.dirname(filepath), m.group(1)))
        else:
            ret += [line]
    return ret

def is_ap_periph(hwdef):
    '''return True if a hwdef is for a AP_Periph board'''
    lines = read_hwdef(hwdef)
    for line in lines:
        if line.find('AP_PERIPH') != -1:
            return True
    return False

def get_board_list():
    '''add boards based on existence of hwdef-bl.dat in any HAL's hwdef subdirectories'''
    board_list = []
    for hal, root in HWDEF_ROOTS:
        if not os.path.isdir(root):
            continue
        dirname, dirlist, filenames = next(os.walk(root))
        for d in dirlist:
            hwdef = os.path.join(dirname, d, 'hwdef-bl.dat')
            if os.path.exists(hwdef):
                if args.periph_only and not is_ap_periph(hwdef):
                    continue
                board_list.append(d)
    return board_list

def validate_signing_keys(keys):
    """Validate that all signing key files exist and are not private keys"""
    missing_keys = []
    private_keys = []

    for key in keys:
        if not os.path.isfile(key):
            missing_keys.append(key)
        elif os.path.basename(key).lower().find("private") != -1:
            private_keys.append(key)

    if missing_keys:
        print("Error: The following files were not found:")
        for key in missing_keys:
            print(f"  {key}")
        sys.exit(1)

    if private_keys:
        print("Error: You must use the public key in the bootloader. Check the following files:")
        for key in private_keys:
            print(f"  {key}")
        sys.exit(1)

if args.signing_key is not None:
    validate_signing_keys(args.signing_key)

def run_program(cmd_list):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("Build failed: %s" % ' '.join(cmd_list))
        return False
    return True


def build_board(board):
    # do not attempt to build a bootloader for boards without hwdef-bl.dat
    if not has_hwdef_bl(board):
        print(f"Skipping {board}: no hwdef-bl.dat (no bootloader for this board)")
        return True  # treat as success, since there is nothing to build

    configure_args = "--board %s --bootloader --no-submodule-update --Werror" % board
    configure_args = configure_args.split()
    if args.signing_key is not None:
        print("Building secure bootloader")
        configure_args.append("--signed-fw")
    if args.debug:
        print("Building debug variant")
        configure_args.append("--debug")
    if args.debug_symbols:
        print("Building with debug symbols")
        configure_args.append("--debug-symbols")
    if not run_program(["./waf", "configure"] + configure_args):
        return False
    if not run_program(["./waf", "clean"]):
        return False
    if not run_program(["./waf", "bootloader"]):
        return False
    return True

board_list = get_board_list()
def get_all_board_dirs():
    dirs = []
    for hal, root in HWDEF_ROOTS:
        if not os.path.isdir(root):
            continue
        dirname, dirlist, filenames = next(os.walk(root))
        dirs += dirlist
    return dirs

all_board_dirs = get_all_board_dirs()

# Determine pattern matches against *all* directories and against buildable boards
matches_all = [b for b in all_board_dirs if fnmatch.fnmatch(b, args.pattern)]
matches_buildable = [b for b in board_list if fnmatch.fnmatch(b, args.pattern)]

# If nothing matches any directory name at all, warn once
if args.pattern != '*' and not matches_all:
    print(f"Warning: no board matches pattern '{args.pattern}'. Continuing.")

# For directories that match the pattern but *aren't* buildable (no hwdef-bl.dat),
# print the explicit skip message now so users see why their requested board didn't build.
for b in matches_all:
    if b not in board_list:
        print(f"Skipping {b}: no hwdef-bl.dat (no bootloader for this board)")

additional_args = []
if args.omit_ardupilot_keys:
    # If the user has requested to omit ardupilot keys, ensure it is forwarded to the make_secure_bl program
    additional_args.append("--omit-ardupilot-keys")

# check that the user-supplied board pattern matches something; if not, warn and exit
for board in board_list:
    if not fnmatch.fnmatch(board, args.pattern):
        continue
    if not has_hwdef_bl(board):
        print(f"Skipping {board}: no hwdef-bl.dat (no bootloader for this board)")
        continue
    print("Building for %s" % board)
    if not build_board(board):
        failed_boards.add(board)
        continue
    bl_file = 'Tools/bootloaders/%s_bl.bin' % board
    hex_file = 'Tools/bootloaders/%s_bl.hex' % board
    elf_file = 'Tools/bootloaders/%s_bl.elf' % board
    hal = board_hal(board)
    if hal == 'Zephyr':
        # Zephyr writes one set of artefacts per board under zephyr_build, and
        # the bootloader build is just another firmware to it.
        built_dir = 'build/%s/zephyr_build/zephyr' % board
        shutil.copy(os.path.join(built_dir, 'zephyr.bin'), bl_file)
        print("Created %s" % bl_file)
        # No ELF: Zephyr links with full debug info, so these run to several MB
        # against about 500 KB for the largest ChibiOS one, and Tools/bootloaders
        # carries an ELF for well under a quarter of its boards anyway.
        elf_file = None
    else:
        built_dir = None
        shutil.copy('build/%s/bin/AP_Bootloader.bin' % board, bl_file)
        print("Created %s" % bl_file)
        shutil.copy('build/%s/bootloader/AP_Bootloader' % board, elf_file)
        print("Created %s" % elf_file)
    if args.signing_key is not None:
        print("Signing bootloader with %s" % ", ".join(args.signing_key))
        if not run_program(["./Tools/scripts/signing/make_secure_bl.py", *additional_args, bl_file] + args.signing_key):
            print("Failed to sign bootloader for %s" % board)
            sys.exit(1)
        if elf_file is not None and not run_program(["./Tools/scripts/signing/make_secure_bl.py", *additional_args, elf_file] + args.signing_key):
            print("Failed to sign ELF bootloader for %s" % board)
            sys.exit(1)
    if built_dir is not None:
        # Take Zephyr's own hex rather than re-deriving one: bin2hex needs the
        # flash base passed in, and it is not 0x08000000 on every Zephyr board -
        # mr_vmu_rt1176 runs from external flash at 0x30000000. Zephyr already
        # emits a hex with the right addresses from the linker script.
        zephyr_hex = os.path.join(built_dir, 'zephyr.hex')
        if not os.path.exists(zephyr_hex):
            print("No %s to copy; skipping hex for %s" % (zephyr_hex, board))
            continue
        shutil.copy(zephyr_hex, hex_file)
    elif not run_program([sys.executable, "Tools/scripts/bin2hex.py", "--offset", "0x08000000", bl_file, hex_file]):
        failed_boards.add(board)
        continue
    print("Created %s" % hex_file)

if len(failed_boards):
    print("Failed boards: %s" % list(failed_boards))

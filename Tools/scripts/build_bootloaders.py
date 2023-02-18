#!/usr/bin/env python

"""
script to build all of our bootloaders using AP_Bootloader and put the resulting binaries in Tools/bootloaders
"""

import os
import shutil
import subprocess
import sys
import fnmatch

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='make_secure_bl')
parser.add_argument("--signing-key", type=str, default=None, help="signing key for secure bootloader")
parser.add_argument("--debug", action='store_true', default=False, help="build with debug symbols")
parser.add_argument("pattern", type=str, default='*', help="board wildcard pattern")
args = parser.parse_args()

if args.signing_key is not None and os.path.basename(args.signing_key).lower().find("private") != -1:
    # prevent the easy mistake of using private key
    print("You must use the public key in the bootloader")
    sys.exit(1)

os.environ['PYTHONUNBUFFERED'] = '1'

failed_boards = set()

def get_board_list():
    '''add boards based on existance of hwdef-bl.dat in subdirectories for ChibiOS'''
    board_list = []
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ChibiOS/hwdef'))
    for d in dirlist:
        hwdef = os.path.join(dirname, d, 'hwdef-bl.dat')
        if os.path.exists(hwdef):
            board_list.append(d)
    return board_list

def run_program(cmd_list):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("Build failed: %s" % ' '.join(cmd_list))
        return False
    return True

def build_board(board):
    configure_args = "--board %s --bootloader --no-submodule-update --Werror" % board
    configure_args = configure_args.split()
    if args.signing_key is not None:
        print("Building secure bootloader")
        configure_args.append("--signed-fw")
    if args.debug:
        print("Building with debug symbols")
        configure_args.append("--debug")
    if not run_program(["./waf", "configure"] + configure_args):
        return False
    if not run_program(["./waf", "clean"]):
        return False
    if not run_program(["./waf", "bootloader"]):
        return False
    return True

for board in get_board_list():
    if not fnmatch.fnmatch(board, args.pattern):
        continue
    print("Building for %s" % board)
    if not build_board(board):
        failed_boards.add(board)
        continue
    bl_file = 'Tools/bootloaders/%s_bl.bin' % board
    hex_file = 'Tools/bootloaders/%s_bl.hex' % board
    elf_file = 'Tools/bootloaders/%s_bl.elf' % board
    shutil.copy('build/%s/bin/AP_Bootloader.bin' % board, bl_file)
    print("Created %s" % bl_file)
    shutil.copy('build/%s/bootloader/AP_Bootloader' % board, elf_file)
    print("Created %s" % elf_file)
    if args.signing_key is not None:
        print("Signing bootloader with %s" % args.signing_key)
        if not run_program(["./Tools/scripts/signing/make_secure_bl.py", bl_file, args.signing_key]):
            print("Failed to sign bootloader for %s" % board)
            sys.exit(1)
        if not run_program(["./Tools/scripts/signing/make_secure_bl.py", elf_file, args.signing_key]):
            print("Failed to sign ELF bootloader for %s" % board)
            sys.exit(1)
    if not run_program([sys.executable, "Tools/scripts/bin2hex.py", "--offset", "0x08000000", bl_file, hex_file]):
        failed_boards.add(board)
        continue
    print("Created %s" % hex_file)

if len(failed_boards):
    print("Failed boards: %s" % list(failed_boards))

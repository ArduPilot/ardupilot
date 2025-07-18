#!/usr/bin/env python3

# flake8: noqa

"""
script to test build all of our peripheral firmwares
"""

import os
import shutil
import subprocess
import sys
import fnmatch
import board_list

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='build_peripherals')
parser.add_argument("pattern", type=str, default='*', help="board wildcard pattern")
parser.add_argument("--debug", action='store_true', default=False, help="build with debug symbols")
parser.add_argument("--stop", action='store_true', default=False, help="stop on a failed build")
parser.add_argument("--configure-only", action='store_true', default=False, help="only run configure")
parser.add_argument("--noclean", action='store_true', default=False, help="don't run waf clean")
args = parser.parse_args()

os.environ['PYTHONUNBUFFERED'] = '1'

failed_boards = set()

def run_program(cmd_list):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("Build failed: %s" % ' '.join(cmd_list))
        return False
    return True

def build_board(board):
    configure_args = "--board %s --no-submodule-update --Werror" % board
    configure_args = configure_args.split()
    if args.debug:
        print("Building with debug symbols")
        configure_args.append("--debug")
    if not run_program(["./waf", "configure"] + configure_args):
        return False
    if args.configure_only:
        return True
    if not args.noclean and not run_program(["./waf", "clean"]):
        return False
    if not run_program(["./waf", "AP_Periph"]):
        return False
    return True

boards = board_list.BoardList().find_ap_periph_boards()

for board in boards:
    if not fnmatch.fnmatch(board, args.pattern):
        continue
    print("Building for %s" % board)
    if not build_board(board):
        failed_boards.add(board)
        if args.stop:
            break
        continue

if len(failed_boards):
    print("Failed boards: %s" % list(failed_boards))
else:
    print("No failed builds")


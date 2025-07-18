#!/usr/bin/env python3

# flake8: noqa

"""
script to run configure for all hwdef.dat, to check for syntax errors
"""

import os
import subprocess
import sys
import fnmatch
import shutil

import argparse

# modify our search path:
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_ChibiOS/hwdef/scripts'))
import chibios_hwdef

parser = argparse.ArgumentParser(description='configure all ChibiOS boards')
parser.add_argument('--build', action='store_true', default=False, help='build as well as configure')
parser.add_argument('--build-target', default='copter', help='build target')
parser.add_argument('--stop', action='store_true', default=False, help='stop on configure or build failure')
parser.add_argument('--no-bl', action='store_true', default=False, help="don't check bootloader builds")
parser.add_argument('--only-bl', action='store_true', default=False, help="only check bootloader builds")
parser.add_argument('--Werror', action='store_true', default=False, help="build with -Werror")
parser.add_argument('--pattern', default='*')
parser.add_argument('--start', default=None, type=int, help='continue from specified build number')
parser.add_argument('--python', default='python')
parser.add_argument('--copy-hwdef-incs-to-directory', default=None, help='directory hwdefs should be copied to')
args = parser.parse_args()

os.environ['PYTHONUNBUFFERED'] = '1'

failures = []
done = []
board_list = []

def get_board_list():
    '''add boards based on existence of hwdef-bl.dat in subdirectories for ChibiOS'''
    board_list = []
    # these are base builds, and don't build directly
    omit = []
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ChibiOS/hwdef'))
    for d in dirlist:
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        if os.path.exists(hwdef) and d not in omit:
            board_list.append(d)
    return board_list

def run_program(cmd_list, build):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("FAILED BUILD: %s %s" % (build, ' '.join(cmd_list)))
        global failures
        failures.append(build)
        if args.stop:
            sys.exit(1)

for board in sorted(get_board_list()):
    if not fnmatch.fnmatch(board, args.pattern):
        continue
    board_list.append(board)

if args.start is not None:
    if args.start < 1 or args.start >= len(board_list):
        print("Invalid start %u for %u boards" % (args.start, len(board_list)))
        sys.exit(1)
    board_list = board_list[args.start-1:]

def is_ap_periph(board):
    hwdef = os.path.join('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat' % board)
    ch = chibios_hwdef.ChibiOSHWDef(hwdef=[hwdef], quiet=True)
    ch.process_hwdefs()
    return ch.is_periph_fw()

if args.copy_hwdef_incs_to_directory is not None:
    os.makedirs(args.copy_hwdef_incs_to_directory)

def handle_hwdef_copy(directory, board, bootloader=False):
    source = os.path.join("build", board, "hwdef.h")
    if bootloader:
        filename = "hwdef-%s-bl.h" % board
    elif board == "iomcu":
        filename = "hwdef-%s-iomcu.h" % board
    elif is_ap_periph(board):
        filename = "hwdef-%s-periph.h" % board
    else:
        filename = "hwdef-%s.h" % board
    target = os.path.join(directory, filename)
    shutil.copy(source, target)

for board in board_list:
    done.append(board)
    print("Configuring for %s [%u/%u failed=%u]" % (board, len(done), len(board_list), len(failures)))
    config_opts = ["--board", board]
    if args.Werror:
        config_opts += ["--Werror"]
    if not args.only_bl:
        run_program([args.python, "waf", "configure"] + config_opts, "configure: " + board)
        if args.copy_hwdef_incs_to_directory is not None:
            handle_hwdef_copy(args.copy_hwdef_incs_to_directory, board)
    if args.build:
        if board == "iomcu":
            target = "iofirmware"
        elif is_ap_periph(board):
            target = "AP_Periph"
        else:
            target = args.build_target
        if target.find('/') != -1:
            run_program([args.python, "waf", "--target", target], "build: " + board)
        else:
            run_program([args.python, "waf", target], "build: " + board)
    if args.no_bl:
        continue
    # check for bootloader def
    hwdef_bl = os.path.join('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef-bl.dat' % board)
    if os.path.exists(hwdef_bl):
        print("Configuring bootloader for %s" % board)
        run_program([args.python, "waf", "configure", "--board", board, "--bootloader"], "configure: " + board + "-bl")
        if args.copy_hwdef_incs_to_directory is not None:
            handle_hwdef_copy(args.copy_hwdef_incs_to_directory, board, bootloader=True)
        if args.build:
            run_program([args.python, "waf", "bootloader"], "build: " + board + "-bl")

if len(failures) > 0:
    print("Failed builds:")
    for f in failures:
        print('  ' + f)
    sys.exit(1)

sys.exit(0)

#!/usr/bin/env python

"""
script to run configre for all hwdef.dat, to check for syntax errors
"""

import os
import subprocess
import sys
import fnmatch
import shutil

import argparse

parser = argparse.ArgumentParser(description='configure all ChibiOS boards')
parser.add_argument('--build', action='store_true', default=False, help='build as well as configure')
parser.add_argument('--build-target', default='copter', help='build target')
parser.add_argument('--stop', action='store_true', default=False, help='stop on configure or build failure')
parser.add_argument('--no-bl', action='store_true', default=False, help="don't check bootloader builds")
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
    '''add boards based on existance of hwdef-bl.dat in subdirectories for ChibiOS'''
    board_list = []
    # these are base builds, and don't build directly
    omit = []
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ChibiOS/hwdef'))
    for d in dirlist:
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        if os.path.exists(hwdef) and not d in omit:
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
    try:
        r = open(hwdef, 'r').read()
        if r.find('periph/hwdef.dat') != -1 or r.find('AP_PERIPH') != -1:
            print("%s is AP_Periph" % board)
            return True
    except Exception as ex:
        pass
    return False

if args.copy_hwdef_incs_to_directory is not None:
    os.makedirs(args.copy_hwdef_incs_to_directory)

for board in board_list:
    done.append(board)
    print("Configuring for %s [%u/%u failed=%u]" % (board, len(done), len(board_list), len(failures)))
    config_opts = ["--board", board]
    if args.Werror:
        config_opts += ["--Werror"]
    run_program([args.python, "waf", "configure"] + config_opts, "configure: " + board)
    if args.copy_hwdef_incs_to_directory is not None:
        source = os.path.join("build", board, "hwdef.h")
        if board == "iomcu":
            filename = "hwdef-%s-iomcu.h" % board
        elif is_ap_periph(board):
            filename = "hwdef-%s-periph.h" % board
        else:
            filename = "hwdef-%s.h" % board
        target = os.path.join(args.copy_hwdef_incs_to_directory, filename)
        shutil.copy(source, target)
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
        if args.build:
            run_program([args.python, "waf", "bootloader"], "build: " + board + "-bl")

if len(failures) > 0:
    print("Failed builds:")
    for f in failures:
        print('  ' + f)
    sys.exit(1)

sys.exit(0)

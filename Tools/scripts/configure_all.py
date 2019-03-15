#!/usr/bin/env python

"""
script to run configre for all hwdef.dat, to check for syntax errors
"""

import os
import shutil
import subprocess
import sys
import fnmatch

import argparse

parser = argparse.ArgumentParser(description='configure all ChibiOS boards')
parser.add_argument('--build', action='store_true', default=False, help='build as well as configure')
parser.add_argument('--stop', action='store_true', default=False, help='stop on build fail')
parser.add_argument('--pattern', default='*')
args = parser.parse_args()

os.environ['PYTHONUNBUFFERED'] = '1'

failures = []

def get_board_list():
    '''add boards based on existance of hwdef-bl.dat in subdirectories for ChibiOS'''
    board_list = []
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ChibiOS/hwdef'))
    for d in dirlist:
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        if os.path.exists(hwdef):
            board_list.append(d)
    return board_list

def run_program(cmd_list, build):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("Build failed: %s %s" % (build, ' '.join(cmd_list)))
        global failures
        failures.append(build)
        if args.stop:
            sys.exit(1)

for board in get_board_list():
    if not fnmatch.fnmatch(board, args.pattern):
        continue
    print("Configuring for %s" % board)
    run_program(["./waf", "configure", "--board", board], "configure: " + board)
    if args.build:
        run_program(["./waf", "copter"], "build: " + board)
    # check for bootloader def
    hwdef_bl = os.path.join('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef-bl.dat' % board)
    if os.path.exists(hwdef_bl):
        print("Configuring bootloader for %s" % board)
        run_program(["./waf", "configure", "--board", board, "--bootloader"], "configure: " + board + "-bl")
        if args.build:
            run_program(["./waf", "bootloader"], "build: " + board + "-bl")

if len(failures) > 0:
    print("Failed builds:")
    for f in failures:
        print('  ' + f)
    sys.exit(1)

sys.exit(0)

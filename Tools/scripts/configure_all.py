#!/usr/bin/env python

"""
script to run configre for all hwdef.dat, to check for syntax errors
"""

import os
import shutil
import subprocess
import sys
import fnmatch

board_pattern = '*'

# allow argument for pattern of boards to build
if len(sys.argv)>1:
    board_pattern = sys.argv[1]

os.environ['PYTHONUNBUFFERED'] = '1'

def get_board_list():
    '''add boards based on existance of hwdef-bl.dat in subdirectories for ChibiOS'''
    board_list = []
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ChibiOS/hwdef'))
    for d in dirlist:
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        if os.path.exists(hwdef):
            board_list.append(d)
    return board_list

def run_program(cmd_list):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("Build failed: %s" % ' '.join(cmd_list))
        sys.exit(1)

for board in get_board_list():
    if not fnmatch.fnmatch(board, board_pattern):
        continue
    print("Configuring for %s" % board)
    run_program(["./waf", "configure", "--board", board])
    # check for bootloader def
    hwdef_bl = os.path.join('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef-bl.dat' % board)
    if os.path.exists(hwdef_bl):
        print("Configuring bootloader for %s" % board)
        run_program(["./waf", "configure", "--board", board, "--bootloader"])

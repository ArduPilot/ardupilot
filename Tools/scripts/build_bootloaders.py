#!/usr/bin/env python

"""
script to build all of our bootloaders using AP_Bootloader and put the resulting binaries in Tools/bootloaders
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
        hwdef = os.path.join(dirname, d, 'hwdef-bl.dat')
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
    print("Building for %s" % board)
    run_program(["./waf", "configure", "--board", board, "--bootloader"])
    run_program(["./waf", "clean"])
    run_program(["./waf", "bootloader"])
    shutil.copy('build/%s/bin/AP_Bootloader.bin' % board, 'Tools/bootloaders/%s_bl.bin' % board)
    run_program(["Tools/scripts/bin2hex.py", "--offset", "0x08000000", 'Tools/bootloaders/%s_bl.bin' % board, 'Tools/bootloaders/%s_bl.hex' % board])
    shutil.copy('build/%s/bootloader/AP_Bootloader' % board, 'Tools/bootloaders/%s_bl.elf' % board)

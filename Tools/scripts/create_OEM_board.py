#!/usr/bin/env python

"""
script to automatically create a copy of a board for an OEM setup
usage example : ./Tools/scripts/create_OEM_board.py mRoPixracerPro mRoPixracerPro-MyCompany

AP_FLAKE8_CLEAN
"""

import sys
import os
import subprocess

import pathlib

board_name = sys.argv[1]
oem_board_name = sys.argv[2]


# directory creation
if not os.path.exists("libraries/AP_HAL_ChibiOS/hwdef/%s" % oem_board_name):
    subprocess.run(["mkdir",  "libraries/AP_HAL_ChibiOS/hwdef/%s" % oem_board_name])
    # create files and add reference to originals
    f = open("libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat" % oem_board_name, "x")
    f.write("include ../%s/hwdef.dat\n" % board_name)
    f.close()
    f = open("libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef-bl.dat" % oem_board_name, "x")
    f.write("include ../%s/hwdef-bl.dat\n" % board_name)
    f.close()
    if os.path.exists("libraries/AP_HAL_ChibiOS/hwdef/%s/defaults.parm" % board_name):
        path = pathlib.Path(f"libraries/AP_HAL_ChibiOS/hwdef/{oem_board_name}/defaults.parm")
        path.write_text(f"@include ../{board_name}/defaults.parm\n")  # noqa

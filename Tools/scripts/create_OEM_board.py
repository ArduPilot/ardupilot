#!/usr/bin/env python

"""
script to automatically create a copy of a board for an OEM setup
usage example : ./Tools/scripts/create_OEM_board.py mRoPixracerPro mRoPixracerPro-MyCompany
"""

import sys
import os
import subprocess

board_name = sys.argv[1]
oem_board_name = sys.argv[2]


# directory creation
if not os.path.exists("libraries/AP_HAL_ChibiOS/hwdef/%s" % oem_board_name):
    subprocess.run(["mkdir",  "libraries/AP_HAL_ChibiOS/hwdef/%s" % oem_board_name])
    # create files and add reference to originals
    f=open("libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat" % oem_board_name, "x")
    f.write("include ../%s/hwdef.dat" % board_name)
    f.close()
    f=open("libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef-bl.dat" % oem_board_name, "x")
    f.write("include ../%s/hwdef-bl.dat" % board_name)
    f.close()
    if os.path.exists("libraries/AP_HAL_ChibiOS/hwdef/%s/defaults.parm" % board_name):
        subprocess.run(["cp", "libraries/AP_HAL_ChibiOS/hwdef/%s/defaults.parm" % board_name, "libraries/AP_HAL_ChibiOS/hwdef/%s/defaults.parm" % oem_board_name])



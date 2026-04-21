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

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_ChibiOS/hwdef/scripts'))
import chibios_hwdef  # noqa


class OEMCreate:
    def __init__(self, oem_board_name, board_name):
        self.oem_board_name = oem_board_name
        self.board_name = board_name

    def run(self):
        hwdef_dir = "libraries/AP_HAL_ChibiOS/hwdef"
        oem_board_dirpath = f"{hwdef_dir}/{oem_board_name}"

        if os.path.exists(oem_board_dirpath):
            raise ValueError(f"{oem_board_dirpath} already exists")  # FIXME exception type

        hwdef_include_relpath = None
        for extension in "dat", "inc":
            tmp = f"../{board_name}/hwdef.{extension}"
            tmp_norm = os.path.normpath(f"{oem_board_dirpath}/{tmp}")
            if os.path.exists(tmp_norm):
                hwdef_include_relpath = tmp
                hwdef_include_normpath = tmp_norm
                break

        hwdef_content = f"include {hwdef_include_relpath}\n"

        ch = chibios_hwdef.ChibiOSHWDef(hwdef_include_normpath)
        use_bootloader_from_board = ch.get_config('USE_BOOTLOADER_FROM_BOARD', default=None, required=False)
        if use_bootloader_from_board is None:
            hwdef_content += "\n"
            hwdef_content += f"USE_BOOTLOADER_FROM_BOARD {board_name}\n"

        subprocess.run(["mkdir",  oem_board_dirpath])

        # create files and add reference to originals
        new_hwdef = pathlib.Path(oem_board_dirpath, "hwdef.dat")

        if hwdef_include_relpath is None:
            raise ValueError(f"Could not find .inc or .dat for {board_name}")

        new_hwdef.write_text(hwdef_content)

        if os.path.exists(f"{hwdef_dir}/{board_name}/defaults.parm"):
            path = pathlib.Path(f"{hwdef_dir}/{oem_board_name}/defaults.parm")
            path.write_text(f"@include ../{board_name}/defaults.parm\n")  # noqa


board_name = sys.argv[1]
oem_board_name = sys.argv[2]

oemcreate = OEMCreate(board_name, oem_board_name)
oemcreate.run()

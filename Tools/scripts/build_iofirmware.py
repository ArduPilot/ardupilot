#!/usr/bin/env python

"""
script to build iofirmware and copy to Tools/IO_Firmware

AP_FLAKE8_CLEAN
"""

import os
import shutil
import subprocess
import sys

os.environ['PYTHONUNBUFFERED'] = '1'


def run_program(cmd_list):
    print("Running (%s)" % " ".join(cmd_list))
    retcode = subprocess.call(cmd_list)
    if retcode != 0:
        print("Build failed: %s" % ' '.join(cmd_list))
        sys.exit(1)


print("Building iofirmware")
run_program(["./waf", "configure", "--board", 'iomcu'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/iomcu/bin/iofirmware_lowpolh.bin', 'Tools/IO_Firmware/iofirmware_lowpolh.bin')
shutil.copy('build/iomcu/bin/iofirmware_highpolh.bin', 'Tools/IO_Firmware/iofirmware_highpolh.bin')

run_program(["./waf", "configure", "--board", 'iomcu_f103_8MHz'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/iomcu_f103_8MHz/bin/iofirmware_lowpolh.bin', 'Tools/IO_Firmware/iofirmware_f103_8MHz_lowpolh.bin')
shutil.copy('build/iomcu_f103_8MHz/bin/iofirmware_highpolh.bin', 'Tools/IO_Firmware/iofirmware_f103_8MHz_highpolh.bin')

#!/usr/bin/env python

"""
script to build iofirmware and copy to Tools/IO_Firmware
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
shutil.copy('build/iomcu/bin/iofirmware.bin', 'Tools/IO_Firmware/fmuv2_IO.bin')

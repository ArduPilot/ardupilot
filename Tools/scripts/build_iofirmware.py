#!/usr/bin/env python3

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

run_program(["./waf", "configure", "--board", 'iomcu', '--enable-iomcu-profiled-support'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/iomcu/bin/iofirmware_lowpolh.bin', 'Tools/IO_Firmware/iofirmware_cube_lowpolh.bin')
shutil.copy('build/iomcu/bin/iofirmware_highpolh.bin', 'Tools/IO_Firmware/iofirmware_cube_highpolh.bin')

run_program(["./waf", "configure", "--board", 'iomcu-dshot'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/iomcu-dshot/bin/iofirmware_lowpolh.bin', 'Tools/IO_Firmware/iofirmware_dshot_lowpolh.bin')
shutil.copy('build/iomcu-dshot/bin/iofirmware_highpolh.bin', 'Tools/IO_Firmware/iofirmware_dshot_highpolh.bin')

run_program(["./waf", "configure", "--board", 'iomcu-dshot', '--enable-iomcu-profiled-support'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/iomcu-dshot/bin/iofirmware_lowpolh.bin', 'Tools/IO_Firmware/iofirmware_cube_dshot_lowpolh.bin')
shutil.copy('build/iomcu-dshot/bin/iofirmware_highpolh.bin', 'Tools/IO_Firmware/iofirmware_cube_dshot_highpolh.bin')

run_program(["./waf", "configure", "--board", 'iomcu-f103'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/iomcu-f103/bin/iofirmware_lowpolh.bin', 'Tools/IO_Firmware/iofirmware_f103_lowpolh.bin')
shutil.copy('build/iomcu-f103/bin/iofirmware_highpolh.bin', 'Tools/IO_Firmware/iofirmware_f103_highpolh.bin')

run_program(["./waf", "configure", "--board", 'iomcu-f103-dshot'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/iomcu-f103-dshot/bin/iofirmware_lowpolh.bin', 'Tools/IO_Firmware/iofirmware_f103_dshot_lowpolh.bin')
shutil.copy('build/iomcu-f103-dshot/bin/iofirmware_highpolh.bin', 'Tools/IO_Firmware/iofirmware_f103_dshot_highpolh.bin')

run_program(["./waf", "configure", "--board", 'iomcu_f103_8MHz'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/iomcu_f103_8MHz/bin/iofirmware_lowpolh.bin', 'Tools/IO_Firmware/iofirmware_f103_8MHz_lowpolh.bin')
shutil.copy('build/iomcu_f103_8MHz/bin/iofirmware_highpolh.bin', 'Tools/IO_Firmware/iofirmware_f103_8MHz_highpolh.bin')

run_program(["./waf", "configure", "--board", 'iomcu-f103-8MHz-dshot'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/iomcu-f103-8MHz-dshot/bin/iofirmware_lowpolh.bin',
            'Tools/IO_Firmware/iofirmware_f103_8MHz_dshot_lowpolh.bin')
shutil.copy('build/iomcu-f103-8MHz-dshot/bin/iofirmware_highpolh.bin',
            'Tools/IO_Firmware/iofirmware_f103_8MHz_dshot_highpolh.bin')

run_program(["./waf", "configure", "--board", 'CubeRedSecondary-IO'])
run_program(["./waf", "clean"])
run_program(["./waf", "iofirmware"])
shutil.copy('build/CubeRedSecondary-IO/bin/iofirmware.bin', 'Tools/IO_Firmware/iofirmware_cubered.bin')

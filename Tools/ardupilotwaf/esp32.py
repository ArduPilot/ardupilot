#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for ESP32 build
"""

from waflib import Errors, Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os
import shutil
import sys
import re
import pickle
import subprocess

def configure(cfg):
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    cfg.find_program('make', var='MAKE')
    env = cfg.env
    env.AP_HAL_PLANE = srcpath('libraries/AP_HAL_ESP32/plane')
    env.AP_HAL_COPTER = srcpath('libraries/AP_HAL_ESP32/copter')
    env.AP_PROGRAM_FEATURES += ['esp32_ap_program']
    try:
        cmd = "cd {0}&&{1} sdkconfig".format(env.AP_HAL_PLANE, env.MAKE[0])
        ret = subprocess.call(cmd, shell=True)
    except Exception as e:
        print e
        cfg.fatal("Failed to configure sdk")
    if ret != 0:
        cfg.fatal("Failed to configure sdk ret=%d" % ret)

class build_esp32_image_plane(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="cd ${AP_HAL_PLANE}&&'${MAKE}' V=1"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_copter(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="cd ${AP_HAL_COPTER}&&'${MAKE}' V=1"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

@feature('esp32_ap_program')
@after_method('process_source')
def esp32_firmware(self):
    self.link_task.always_run = True
    if str(self.link_task.outputs[0]).endswith('libarduplane.a'):
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduPlane_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libarduplane.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-plane/arduplane.elf')
        generate_bin_task = self.create_task('build_esp32_image_plane', src=src_in, tgt=img_out)
        generate_bin_task.set_run_after(self.link_task)
    if str(self.link_task.outputs[0]).endswith('libarducopter.a'):
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduCopter_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libarducopter.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-copter/arducopter.elf')
        generate_bin_task = self.create_task('build_esp32_image_copter', src=src_in, tgt=img_out)
        generate_bin_task.set_run_after(self.link_task)
        
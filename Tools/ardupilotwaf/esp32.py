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

def parse_inc_dir(lines):
    for line in lines.splitlines():
        if line.startswith('INCLUDES: '):
            return line.replace('INCLUDES: ', '').split()

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
    if str(self.link_task.outputs[0]).endswith('libarduplane.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduPlane_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libarduplane.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-plane/arduplane.elf')
        generate_bin_task = self.create_task('build_esp32_image_plane', src=src_in, tgt=img_out)        
        generate_bin_task.set_run_after(self.link_task)
        
        #add generated include files
        cmd = "cd {0}&&{1} showinc".format(self.env.AP_HAL_PLANE, self.env.MAKE[0])
        result = subprocess.check_output(cmd, shell=True)
        self.env.INCLUDES += parse_inc_dir(result)
    if str(self.link_task.outputs[0]).endswith('libarducopter.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduCopter_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libarducopter.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-copter/arducopter.elf')
        generate_bin_task = self.create_task('build_esp32_image_copter', src=src_in, tgt=img_out)
        generate_bin_task.set_run_after(self.link_task)
        
        #add generated include files
        cmd = "cd {0}&&{1} showinc".format(self.env.AP_HAL_COPTER, self.env.MAKE[0])
        result = subprocess.check_output(cmd, shell=True)
        self.env.INCLUDES += parse_inc_dir(result)
        
        
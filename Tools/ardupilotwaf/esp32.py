# encoding: utf-8

# flake8: noqa

"""
Waf tool for ESP32 build
"""

from waflib import Build, ConfigSet, Configure, Context, Task, Utils
from waflib import Errors, Logs
from waflib.TaskGen import before, after_method, before_method, feature
from waflib.Configure import conf
from collections import OrderedDict

import os
import shutil
import sys
import traceback
import re
import pickle
import subprocess

import hal_common

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_ESP32/hwdef/scripts'))
import esp32_hwdef  # noqa:501

def configure(cfg):
    mcu_esp32s3 = True if (cfg.variant[0:7] == "esp32s3") else False
    target = "esp32s3" if mcu_esp32s3 else "esp32"
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    def bldpath(path):
        return bldnode.make_node(path).abspath()

    #Load cmake builder and make
    cfg.load('cmake')

    #define env and location for the cmake esp32 file
    env = cfg.env
    env.AP_HAL_ESP32 = srcpath('libraries/AP_HAL_ESP32/targets/'+target+'/esp-idf')
    env.AP_PROGRAM_FEATURES += ['esp32_ap_program']

    env.ESP_IDF_PREFIX_REL = 'esp-idf'

    prefix_node = bldnode.make_node(env.ESP_IDF_PREFIX_REL)
    env.ESP32_TARGET = target
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')

    #Check if esp-idf env are loaded, or load it
    try:
        env.IDF = os.environ['IDF_PATH']
    except:
        env.IDF = cfg.srcnode.abspath()+"/modules/esp_idf"
    print("USING EXPRESSIF IDF:"+str(env.IDF))

    try:
        hwdef_obj = generate_hwdef_h(env)
    except Exception:
        traceback.print_exc()
        cfg.fatal("Failed to process hwdef.dat")
    hal_common.process_hwdef_results(cfg, hwdef_obj)

def generate_hwdef_h(env):
    '''run esp32_hwdef.py'''
    hwdef_dir = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ESP32/hwdef')

    if len(env.HWDEF) == 0:
        env.HWDEF = os.path.join(hwdef_dir, env.BOARD, 'hwdef.dat')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    hwdef = [env.HWDEF]
    if env.HWDEF_EXTRA:
        hwdef.append(env.HWDEF_EXTRA)

    hwdef_obj = esp32_hwdef.ESP32HWDef(
        outdir=hwdef_out,
        hwdef=hwdef,
        quiet=False,
    )
    hwdef_obj.run()

    return hwdef_obj

# delete the output sdkconfig file when the input defaults changes. we take the
# stamp as the output so we can compute the path to the sdkconfig, yet it
# doesn't have to exist when we're done.
class clean_sdkconfig(Task.Task):
    def keyword(self):
        return "delete sdkconfig generated from"

    def run(self):
        prefix = ".clean-stamp-"
        for out in self.outputs:
            if not out.name.startswith(prefix):
                raise ValueError("not a stamp file: "+out)
            dest = out.parent.abspath()+"/"+out.name[len(prefix):]
            if os.path.exists(dest):
                os.unlink(dest)

            # waf needs the output to exist after the task, so touch it
            open(out.abspath(), "w").close()

def pre_build(self):
    """Configure esp-idf as lib target"""
    lib_vars = OrderedDict()
    lib_vars['ARDUPILOT_CMD'] = self.cmd
    lib_vars['WAF_BUILD_TARGET'] = self.targets
    lib_vars['ARDUPILOT_LIB'] = self.bldnode.find_or_declare('lib/').abspath()
    lib_vars['ARDUPILOT_BIN'] = self.bldnode.find_or_declare('lib/bin').abspath()
    target = self.env.ESP32_TARGET
    esp_idf = self.cmake(
            name='esp-idf',
            cmake_vars=lib_vars,
            cmake_src='libraries/AP_HAL_ESP32/targets/'+target+'/esp-idf',
            cmake_bld='esp-idf_build',
            )

    esp_idf_showinc = esp_idf.build('showinc', target='esp-idf_build/includes.list')

    # task to delete the sdkconfig (thereby causing it to be regenerated) when
    # the .defaults changes. it uses a stamp to find the sdkconfig. changing
    # the sdkconfig WILL NOT cause it to be deleted as it's not an input. this
    # is by design so the user can tweak it for testing purposes.
    clean_sdkconfig_task = esp_idf_showinc.create_task("clean_sdkconfig",
        src=self.srcnode.find_or_declare(self.env.AP_HAL_ESP32+"/sdkconfig.defaults"),
        tgt=self.bldnode.find_or_declare("esp-idf_build/.clean-stamp-sdkconfig"))

    esp_idf_showinc.post()

    # ensure the sdkconfig will be deleted before the cmake configure occurs
    # that regenerates it
    esp_idf_showinc.cmake_config_task.set_run_after(clean_sdkconfig_task)

    from waflib import Task
    class load_generated_includes(Task.Task):
        """After includes.list generated include it in env"""
        always_run = True
        def run(tsk):
            bld = tsk.generator.bld
            includes = bld.bldnode.find_or_declare('esp-idf_build/includes.list').read().split()
            #print(includes)
            bld.env.prepend_value('INCLUDES', includes)

    tsk = load_generated_includes(env=self.env)
    tsk.set_inputs(self.path.find_resource('esp-idf_build/includes.list'))
    self.add_to_group(tsk)

@feature('esp32_ap_program')
@after_method('process_source')
def esp32_firmware(self):
    self.link_task.always_run = True
    esp_idf = self.bld.cmake('esp-idf')

    build = esp_idf.build('all', target='esp-idf_build/ardupilot.bin')
    build.post()

    build.cmake_build_task.set_run_after(self.link_task)

    # optional upload is last
    if self.bld.options.upload:
        flasher = esp_idf.build('flash')
        flasher.post()

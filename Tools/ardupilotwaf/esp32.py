# encoding: utf-8

"""
Waf tool for ESP32 and ESP32S3 build/s that are using a hwdef.dat
"""

from waflib import Errors, Logs, Task, Utils, Context, Build, ConfigSet, Configure
from waflib.TaskGen import before, after_method, before_method, feature
from waflib.Configure import conf
from collections import OrderedDict

import os
import shutil
import sys
import re
import pickle
import struct
import subprocess

def configure(cfg):
    mcu_esp32s3 = True if (cfg.variant[0:7] == "esp32s3") else False
    target = "esp32s3" if mcu_esp32s3 else "esp32"
    bldnode = cfg.bldnode.make_node(cfg.variant)
    #print("bldnode:",bldnode,"-------------------------------------------------------------------")
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    def bldpath(path):
        return bldnode.make_node(path).abspath()

    #Load cmake builder and make
    cfg.load('cmake')

    if 's3' in cfg.variant:
        cfg.s3 = True
        cfg.env.s3 = True

    #define env and location for the cmake esp32 file
    env = cfg.env
    env.AP_HAL_ESP32 = srcpath('libraries/AP_HAL_ESP32/targets/'+target+'/esp-idf')
    env.AP_PROGRAM_FEATURES += ['esp32_ap_program']

    if not 's3' in cfg.variant:
        env.ESP_IDF_PREFIX_REL = 'esp-idf'
        prefix_node = bldnode.make_node(env.ESP_IDF_PREFIX_REL)
    else:
        env.ESP_IDF_PREFIX_REL_S3 = 'esp-idf'
        prefix_node = bldnode.make_node(env.ESP_IDF_PREFIX_REL_S3)

    prefix_node = bldnode.make_node(env.ESP_IDF_PREFIX_REL)
    env.ESP32_TARGET = target
    env.BUILDROOT = bldpath('')
    print("env.BUILDROOT:",env.BUILDROOT)

    env.SRCROOT = srcpath('')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')

    #Check if esp-idf env are loaded, or load it
    if not 's3' in cfg.variant:
        env.IDF = cfg.srcnode.abspath()+"/modules/esp_idf"
    else:
        env.IDF = cfg.srcnode.abspath()+"/modules/esp_idf"

    print("USING EXPRESSIF IDF:"+str(env.IDF))

    try:
        env.DEFAULT_PARAMETERS = os.environ['DEFAULT_PARAMETERS']
    except:
        env.DEFAULT_PARAMETERS = cfg.srcnode.abspath()+"/libraries/AP_HAL_ESP32/boards/defaults.parm"
    print("USING DEFAULT_PARAMETERS:"+str(env.DEFAULT_PARAMETERS))

    try:
        ret = _generate_hwdef_h(env)
    except Exception:
        cfg.fatal("Failed to process hwdef.dat")
    if ret != 0:
        cfg.fatal("Failed to process hwdef.dat ret=%d" % ret)
    #env.append_value('GIT_SUBMODULES', 'esp_idf')

    load_env_vars(cfg.env)

def _generate_hwdef_h(env):
    '''run esp32_hwdef.py'''
    import subprocess

    if len(env.HWDEF) == 0:
        env.HWDEF = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ESP32/hwdef/%s/hwdef.dat' % env.BOARD)
    env.BOOTLOADER_OPTION=""

    hwdef_script = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ESP32/hwdef/scripts/esp32_hwdef.py')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    python = sys.executable
    cmd = "{0} '{1}' -D '{2}' --params '{3}' '{4}'".format(python, hwdef_script, hwdef_out, env.DEFAULT_PARAMETERS, env.HWDEF)
    # if env.HWDEF_EXTRA:
    #     cmd += " '{0}'".format(env.HWDEF_EXTRA)
    # if env.BOOTLOADER_OPTION:
    #     cmd += " " + env.BOOTLOADER_OPTION
    print("BUZZ generate hwdef.h from .dat",cmd)
    return subprocess.call(cmd, shell=True)

def pre_build(bld):
    """Configure esp-idf as lib target"""
    #print(bld.env)
    load_env_vars(bld.env)

    print("bld.env.s3",bld.env.s3)

    if bld.env.HAL_NUM_CAN_IFACES:
        bld.get_board().with_can = True

    bld.env.ROMFS_FILES = list(set(bld.env.ROMFS_FILES)) #dedupe
    print("esp32.py: pre_build ROMFS_FILES?=",bld.env.ROMFS_FILES)
    if bld.env.ROMFS_FILES:
            romfs_src = [bld.bldnode.find_or_declare('ap_romfs_embedded.h')]

    lib_vars = OrderedDict()
    lib_vars['ARDUPILOT_CMD'] = bld.cmd
    lib_vars['WAF_BUILD_TARGET'] = bld.targets
    lib_vars['ARDUPILOT_LIB'] = bld.bldnode.find_or_declare('lib/').abspath()
    lib_vars['ARDUPILOT_BIN'] = bld.bldnode.find_or_declare('lib/bin').abspath()
    target = bld.env.ESP32_TARGET
    esp_idf = bld.cmake(
            name='esp-idf',
            cmake_vars=lib_vars,
            cmake_src='libraries/AP_HAL_ESP32/targets/'+target+'/esp-idf',
            cmake_bld='esp-idf_build',
            )
    esp_idf_showinc = esp_idf.build('showinc', target='esp-idf_build/includes.list')
    esp_idf_showinc.post()

    from waflib import Task
    class load_generated_includes(Task.Task):
        """After includes.list generated include it in env"""
        always_run = True
        def run(tsk):
            bld = tsk.generator.bld
            includes = bld.bldnode.find_or_declare('esp-idf_build/includes.list').read().split()
            bld.env.prepend_value('INCLUDES', includes)
    class load_generated_includes_s3(Task.Task):
        """After includes.list generated include it in env"""
        always_run = True
        def run(tsk):
            bld = tsk.generator.bld
            includes_s3 = bld.bldnode.find_or_declare('esp-idf_build/includes.list').read().split()
            bld.env.prepend_value('INCLUDES', includes_s3)

    if not bld.env.s3:
        tsk = load_generated_includes(env=bld.env)
        tsk.set_inputs(bld.path.find_resource('esp-idf_build/includes.list'))
        bld.add_to_group(tsk)

    if bld.env.s3:
        tsk = load_generated_includes_s3(env=bld.env)
        tsk.set_inputs(bld.path.find_resource('esp-idf_build/includes.list'))
        bld.add_to_group(tsk)


# this command takes the bottom THREE .a files and smashes them together into one .a file. 
# libraries/AP_HAL_ESP32/utils/periph.mri contains:
# create bin/AP_Periph.a
# addlib lib/bin/libAP_Periph.a
# addlib lib/libAP_Periph_libs.a
# addlib Tools/AP_Periph/liblibcanard.a
# save
# end
class build_esp32_image_periph(Task.Task):
    color='CYAN'
    run_str="xtensa-esp32-elf-ar -M < ../../libraries/AP_HAL_ESP32/utils/periph.mri ; cp bin/AP_Periph.a lib/AP_Periph.a"
    always_run = True
    def keyword(self):
        return "turning classic periph triple-libs into a single one with AR and custom target...\n"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)
class build_esp32s3_image_periph(Task.Task):
    color='CYAN'
    run_str="xtensa-esp32s3-elf-ar -M < ../../libraries/AP_HAL_ESP32/utils/periph.mri ; cp bin/AP_Periph.a lib/AP_Periph.a"
    always_run = True
    def keyword(self):
        return "turning s3 periph triple-libs into a single one with AR and custom target...\n"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

@feature('esp32_ap_program')
@after_method('process_source')
def esp32_firmware(self):
    self.link_task.always_run = True

    if self.bld.env.s3:
        esp_idf = self.bld.cmake('esp-idf')
        build = esp_idf.build('all', target='esp-idf_build/ardupilot.bin')
    else:
        esp_idf = self.bld.cmake('esp-idf')
        build = esp_idf.build('all', target='esp-idf_build/ardupilot.bin')

    build.post()

    #for periph only, this is a 
    bin_target = self.bld.bldnode.find_or_declare('bin/periph.bin')
    # periph:
    print("esp32_firmware:",self.link_task.outputs[0])
    if str(self.link_task.outputs[0]).endswith('libAP_Periph.a'):
        src_in = [self.bld.bldnode.find_or_declare('lib/libAP_Periph_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libAP_Periph.a')]
        img_out0 = self.bld.bldnode.find_or_declare('bin/AP_Periph.a')
        if self.bld.env.s3:
            self.generate_bin_task = self.create_task('build_esp32s3_image_periph', src=src_in, tgt=img_out0)
        else:
            self.generate_bin_task = self.create_task('build_esp32_image_periph', src=src_in, tgt=img_out0)
            
        self.generate_bin_task.set_run_after(self.link_task)
        build.cmake_build_task.set_run_after(self.generate_bin_task)

    else:
        build.cmake_build_task.set_run_after(self.link_task)
    #


    # tool that can update the default params in a .bin or .apj
    #self.default_params_task = self.create_task('set_default_parameters',
    #                                          src='esp-idf_build/ardupilot.bin')
    #self.default_params_task.set_run_after(self.generate_bin_task)

    # optional upload is last
    if self.bld.options.upload:
        flasher = esp_idf.build('flash')
        flasher.post()


class set_default_parameters(Task.Task):
    color='CYAN'
    always_run = True
    def keyword(self):
        return "setting default params"
    def run(self):

        # TODO: disabled this task outright as apjtool appears to destroy checksums and/or the esp32 partition table
        # TIP:  if u do try this, afterwards, be sure to 'rm -rf build/esp32buzz/idf-plane/*.bin' and re-run waf
        return

def load_env_vars(env):

    # if env.S3:
    #     pass
    '''optionally load extra environment variables from env.py in the build directory'''
    # right now the esp32 build doesn't actually use this, afaict, except for flagging periph builds?
    print("Checking for env.py",env.BUILDROOT)
    env_py = os.path.join(env.BUILDROOT, 'env.py')
    if not os.path.exists(env_py):
        print("No env.py found")
        return
    e = pickle.load(open(env_py, 'rb'))
    for k in e.keys():
        v = e[k]
        #print("BUZZ en loaded:",v,k,e[k])
        if k == 'ROMFS_FILES':
            env.ROMFS_FILES += v
            continue
        if k in env:
            if isinstance(env[k], dict):
                a = v.split('=')
                env[k][a[0]] = '='.join(a[1:])
                print("env updated %s=%s" % (k, v))
            elif isinstance(env[k], list):
                env[k].append(v)
                print("env appended %s=%s" % (k, v))
            else:
                env[k] = v
                print("env added %s=%s" % (k, v))
        else:
            env[k] = v
            print("env set %s=%s" % (k, v))


#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for ChibiOS build
"""

from waflib import Errors, Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os
import shutil
import sys
import re
import pickle

_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/ChibiOS')
    tmp_str = bldnode.find_node('include_dirs').read()
    tmp_str = tmp_str.replace(';\n','')
    tmp_str = tmp_str.replace('-I','')  #remove existing -I flags
    # split, coping with separator
    idirs = re.split('; ', tmp_str)

    # create unique list, coping with relative paths
    idirs2 = []
    for d in idirs:
        if d.startswith('../'):
            # relative paths from the make build are relative to BUILDROOT
            d = os.path.join(bld.env.BUILDROOT, d)
        d = os.path.normpath(d)
        if not d in idirs2:
            idirs2.append(d)
    _dynamic_env_data['include_dirs'] = idirs2

@feature('ch_ap_library', 'ch_ap_program')
@before_method('process_source')
def ch_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    if not _dynamic_env_data:
        _load_dynamic_env_data(self.bld)
    self.use += ' ch'
    self.env.append_value('INCLUDES', _dynamic_env_data['include_dirs'])


class upload_fw(Task.Task):
    color='BLUE'
    always_run = True
    def run(self):
        upload_tools = self.env.get_flat('UPLOAD_TOOLS')
        src = self.inputs[0]
        print("Uploading at baudrate %s" % self.env.UPLOAD_BAUD)
        return self.exec_command("{} '{}/uploader.py' '{}' --baud-bootloader='{}' --baud-bootloader-flash='{}'".format(self.env.get_flat('PYTHON'), upload_tools, src, self.env.UPLOAD_BAUD, self.env.UPLOAD_BAUD))

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(upload_fw, self).exec_command(cmd, **kw)

    def keyword(self):
        return "Uploading"

class set_default_parameters(Task.Task):
    color='CYAN'
    always_run = True
    def keyword(self):
        return "apj_tool"
    def run(self):
        rel_default_parameters = self.env.get_flat('DEFAULT_PARAMETERS')
        abs_default_parameters = os.path.join(self.env.SRCROOT, rel_default_parameters)
        apj_tool = self.env.APJ_TOOL
        sys.path.append(os.path.dirname(apj_tool))
        from apj_tool import embedded_defaults
        defaults = embedded_defaults(self.inputs[0].abspath())
        if defaults.find():
            defaults.set_file(abs_default_parameters)
            defaults.save()


class generate_bin(Task.Task):
    color='CYAN'
    run_str="${OBJCOPY} -O binary ${SRC} ${TGT}"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class generate_apj(Task.Task):
    '''generate an apj firmware file'''
    color='CYAN'
    always_run = True
    def keyword(self):
        return "apj_gen"
    def run(self):
        import json, time, base64, zlib
        img = open(self.inputs[0].abspath(),'rb').read()
        d = {
            "board_id": int(self.env.APJ_BOARD_ID),
            "magic": "APJFWv1",
            "description": "Firmware for a %s board" % self.env.APJ_BOARD_TYPE,
            "image": base64.b64encode(zlib.compress(img,9)).decode('utf-8'),
            "build_time": int(time.time()),
            "summary": self.env.BOARD,
            "version": "0.1",
            "image_size": len(img),
            "git_identity": self.generator.bld.git_head_hash(short=True),
            "board_revision": 0
        }
        apj_file = self.outputs[0].abspath()
        f = open(apj_file, "w")
        f.write(json.dumps(d, indent=4))
        f.close()

class build_abin(Task.Task):
    '''build an abin file for skyviper firmware upload via web UI'''
    color='CYAN'
    run_str='${TOOLS_SCRIPTS}/make_abin.sh ${SRC}.bin ${SRC}.abin'
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_intel_hex(Task.Task):
    '''build an intel hex file for upload with DFU'''
    color='CYAN'
    run_str='${TOOLS_SCRIPTS}/make_intel_hex.py ${SRC} ${FLASH_RESERVE_START_KB}'
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

@feature('ch_ap_program')
@after_method('process_source')
def chibios_firmware(self):
    self.link_task.always_run = True

    link_output = self.link_task.outputs[0]

    bin_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.bin').name)
    apj_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.apj').name)

    generate_bin_task = self.create_task('generate_bin', src=link_output, tgt=bin_target)
    generate_bin_task.set_run_after(self.link_task)

    generate_apj_task = self.create_task('generate_apj', src=bin_target, tgt=apj_target)
    generate_apj_task.set_run_after(generate_bin_task)

    if self.env.BUILD_ABIN:
        abin_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.abin').name)
        abin_task = self.create_task('build_abin', src=link_output, tgt=abin_target)
        abin_task.set_run_after(generate_apj_task)

    bootloader_bin = self.bld.srcnode.make_node("Tools/bootloaders/%s_bl.bin" % self.env.BOARD)
    if os.path.exists(bootloader_bin.abspath()) and self.bld.env.HAVE_INTEL_HEX:
        hex_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.hex').name)
        hex_task = self.create_task('build_intel_hex', src=[bin_target, bootloader_bin], tgt=hex_target)
        hex_task.set_run_after(generate_bin_task)

    if self.env.DEFAULT_PARAMETERS:
        default_params_task = self.create_task('set_default_parameters',
                                               src=link_output)
        default_params_task.set_run_after(self.link_task)
        generate_bin_task.set_run_after(default_params_task)
    
    if self.bld.options.upload:
        self.env.UPLOAD_BAUD = "115200"
        _upload_task = self.create_task('upload_fw', src=apj_target)
        _upload_task.set_run_after(generate_apj_task)

    if self.bld.options.uploadfast:
        self.env.UPLOAD_BAUD = "921600"
        _upload_task = self.create_task('upload_fw', src=apj_target)
        _upload_task.set_run_after(generate_apj_task)

def setup_can_build(cfg):
    '''enable CAN build. By doing this here we can auto-enable CAN in
    the build based on the presence of CAN pins in hwdef.dat'''
    env = cfg.env
    env.AP_LIBRARIES += [
        'AP_UAVCAN',
        'modules/uavcan/libuavcan/src/**/*.cpp',
        'modules/uavcan/libuavcan_drivers/stm32/driver/src/*.cpp'
        ]

    env.CFLAGS += ['-DUAVCAN_STM32_CHIBIOS=1',
                   '-DUAVCAN_STM32_NUM_IFACES=2']

    env.CXXFLAGS += [
        '-Wno-error=cast-align',
        '-DUAVCAN_STM32_CHIBIOS=1',
        '-DUAVCAN_STM32_NUM_IFACES=2'
        ]

    env.DEFINES += [
        'UAVCAN_CPP_VERSION=UAVCAN_CPP03',
        'UAVCAN_NO_ASSERTIONS=1',
        'UAVCAN_NULLPTR=nullptr'
        ]

    env.INCLUDES += [
        cfg.srcnode.find_dir('modules/uavcan/libuavcan/include').abspath(),
        cfg.srcnode.find_dir('modules/uavcan/libuavcan_drivers/stm32/driver/include').abspath()
        ]
    cfg.get_board().with_uavcan = True

def load_env_vars(env):
    '''optionally load extra environment variables from env.py in the build directory'''
    print("Checking for env.py")
    env_py = os.path.join(env.BUILDROOT, 'env.py')
    if not os.path.exists(env_py):
        print("No env.py found")
        return
    e = pickle.load(open(env_py, 'rb'))
    for k in e.keys():
        v = e[k]
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
    if env.ENABLE_ASSERTS:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_ASSERTS=yes'

def setup_optimization(env):
    '''setup optimization flags for build'''
    if env.DEBUG:
        OPTIMIZE = "-Og"
    elif env.OPTIMIZE:
        OPTIMIZE = env.OPTIMIZE
    else:
        OPTIMIZE = "-Os"
    env.CFLAGS += [ OPTIMIZE ]
    env.CXXFLAGS += [ OPTIMIZE ]
    env.CHIBIOS_BUILD_FLAGS += ' USE_COPT=%s' % OPTIMIZE

def configure(cfg):
    cfg.find_program('make', var='MAKE')
    #cfg.objcopy = cfg.find_program('%s-%s'%(cfg.env.TOOLCHAIN,'objcopy'), var='OBJCOPY', mandatory=True)
    cfg.find_program('arm-none-eabi-objcopy', var='OBJCOPY')
    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.AP_PROGRAM_FEATURES += ['ch_ap_program']

    kw = env.AP_LIBRARIES_OBJECTS_KW
    kw['features'] = Utils.to_list(kw.get('features', [])) + ['ch_ap_library']

    env.CH_ROOT = srcpath('modules/ChibiOS')
    env.AP_HAL_ROOT = srcpath('libraries/AP_HAL_ChibiOS')
    env.BUILDDIR = bldpath('modules/ChibiOS')
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.PT_DIR = srcpath('Tools/ardupilotwaf/chibios/image')
    env.MKFW_TOOLS = srcpath('Tools/ardupilotwaf')
    env.UPLOAD_TOOLS = srcpath('Tools/scripts')
    env.CHIBIOS_SCRIPTS = srcpath('libraries/AP_HAL_ChibiOS/hwdef/scripts')
    env.TOOLS_SCRIPTS = srcpath('Tools/scripts')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')
    env.SERIAL_PORT = srcpath('/dev/serial/by-id/*_STLink*')

    # relative paths to pass to make, relative to directory that make is run from
    env.CH_ROOT_REL = os.path.relpath(env.CH_ROOT, env.BUILDROOT)
    env.AP_HAL_REL = os.path.relpath(env.AP_HAL_ROOT, env.BUILDROOT)
    env.BUILDDIR_REL = os.path.relpath(env.BUILDDIR, env.BUILDROOT)

    mk_custom = srcpath('libraries/AP_HAL_ChibiOS/hwdef/%s/chibios_board.mk' % env.BOARD)
    mk_common = srcpath('libraries/AP_HAL_ChibiOS/hwdef/common/chibios_board.mk')
    # see if there is a board specific make file
    if os.path.exists(mk_custom):
        env.BOARD_MK = mk_custom
    else:
        env.BOARD_MK = mk_common

    if cfg.options.default_parameters:
        cfg.msg('Default parameters', cfg.options.default_parameters, color='YELLOW')
        env.DEFAULT_PARAMETERS = srcpath(cfg.options.default_parameters)

    # we need to run chibios_hwdef.py at configure stage to generate the ldscript.ld
    # that is needed by the remaining configure checks
    import subprocess

    if env.BOOTLOADER:
        env.HWDEF = srcpath('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef-bl.dat' % env.BOARD)
        env.BOOTLOADER_OPTION="--bootloader"
    else:
        env.HWDEF = srcpath('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat' % env.BOARD)
        env.BOOTLOADER_OPTION=""
    hwdef_script = srcpath('libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    python = sys.executable
    try:
        cmd = "{0} '{1}' -D '{2}' '{3}' {4}".format(python, hwdef_script, hwdef_out, env.HWDEF, env.BOOTLOADER_OPTION)
        ret = subprocess.call(cmd, shell=True)
    except Exception:
        cfg.fatal("Failed to process hwdef.dat")
    if ret != 0:
        cfg.fatal("Failed to process hwdef.dat ret=%d" % ret)

    load_env_vars(cfg.env)
    if env.HAL_WITH_UAVCAN:
        setup_can_build(cfg)
    setup_optimization(cfg.env)

def pre_build(bld):
    '''pre-build hook to change dynamic sources'''
    load_env_vars(bld.env)
    if bld.env.HAL_WITH_UAVCAN:
        bld.get_board().with_uavcan = True

def build(bld):

    bld(
        # build hwdef.h from hwdef.dat. This is needed after a waf clean
        source=bld.path.ant_glob(bld.env.HWDEF),
        rule="%s '${AP_HAL_ROOT}/hwdef/scripts/chibios_hwdef.py' -D '${BUILDROOT}' '%s' %s" % (
            bld.env.get_flat('PYTHON'), bld.env.HWDEF, bld.env.BOOTLOADER_OPTION),
        group='dynamic_sources',
        target=[bld.bldnode.find_or_declare('hwdef.h'),
                bld.bldnode.find_or_declare('ldscript.ld')]
    )
    
    bld(
        # create the file modules/ChibiOS/include_dirs
        rule="touch Makefile && BUILDDIR=${BUILDDIR_REL} CHIBIOS=${CH_ROOT_REL} AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${MAKE} pass -f '${BOARD_MK}'",
        group='dynamic_sources',
        target=bld.bldnode.find_or_declare('modules/ChibiOS/include_dirs')
    )

    common_src = [bld.bldnode.find_or_declare('hwdef.h'),
                  bld.bldnode.find_or_declare('modules/ChibiOS/include_dirs')]
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.[ch]')
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.mk')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.[ch]')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.mk')
    if bld.env.ROMFS_FILES:
        common_src += [bld.bldnode.find_or_declare('ap_romfs_embedded.h')]
    ch_task = bld(
        # build libch.a from ChibiOS sources and hwdef.h
        rule="BUILDDIR='${BUILDDIR_REL}' CHIBIOS='${CH_ROOT_REL}' AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} '${MAKE}' lib -f '${BOARD_MK}'",
        group='dynamic_sources',
        source=common_src,
        target=bld.bldnode.find_or_declare('modules/ChibiOS/libch.a')
    )
    ch_task.name = "ChibiOS_lib"

    bld.env.LIB += ['ch']
    bld.env.LIBPATH += ['modules/ChibiOS/']
    wraplist = ['strerror_r', 'fclose', 'freopen', 'fread', 'fprintf', 'sscanf', 'snprintf']
    for w in wraplist:
        bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]

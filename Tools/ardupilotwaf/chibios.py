#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for PX4 build
"""

from waflib import Errors, Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os
import shutil
import sys
import re
_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/ChibiOS')
    tmp_str = bldnode.find_node('include_dirs').read()
    tmp_str = tmp_str.replace(';\n','')
    if 'include_dirs' == 'include_dirs':
        tmp_str = tmp_str.replace('-I','')  #remove existing -I flags
    _dynamic_env_data['include_dirs'] = re.split('; ', tmp_str)
    #print _dynamic_env_data['include_dirs']

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
        return self.exec_command("python {}/px_uploader.py --port /dev/serial/by-id/usb-3D*,/dev/serial/by-id/usb-Ardu* \
                    --baud-bootloader 115200 {}".format(
                   upload_tools, src))

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(upload_fw, self).exec_command(cmd, **kw)

    def keyword(self):
        return "Uploading"

class set_default_parameters(Task.Task):
    color='CYAN'
    run_str='python ${APJ_TOOL} --set-file ${DEFAULT_PARAMETERS} ${SRC}'
    always_run = True
    def keyword(self):
        return "apj_tool"

class generate_fw(Task.Task):
    color='CYAN'
    run_str='${OBJCOPY} -O binary ${SRC} ${SRC}.bin && \
    python ${UPLOAD_TOOLS}/px_mkfw.py --image ${SRC}.bin \
    --prototype ${BUILDROOT}/apj.prototype > ${TGT}'
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
    self.objcopy_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.apj').name)

    generate_fw_task = self.create_task('generate_fw',
                            src=link_output,
                            tgt=self.objcopy_target)
    generate_fw_task.set_run_after(self.link_task)

    if self.env.DEFAULT_PARAMETERS:
        default_params_task = self.create_task('set_default_parameters',
                                               src=link_output)
        default_params_task.set_run_after(self.link_task)
        generate_fw_task.set_run_after(default_params_task)
    
    if self.bld.options.upload:
        _upload_task = self.create_task('upload_fw',
                                src=self.objcopy_target)
        _upload_task.set_run_after(generate_fw_task)

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
    env.PT_DIR = srcpath('Tools/ardupilotwaf/chibios/image')
    env.UPLOAD_TOOLS = srcpath('Tools/ardupilotwaf')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')
    env.SERIAL_PORT = srcpath('/dev/serial/by-id/*_STLink*')

    if cfg.options.default_parameters:
        cfg.msg('Default parameters', cfg.options.default_parameters, color='YELLOW')
        env.DEFAULT_PARAMETERS = srcpath(cfg.options.default_parameters)

    

def build(bld):
    bld(
        # build hwdef.h and apj.prototype from hwdef.dat
        source='libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat' % bld.env.get_flat('BOARD'),
        rule='python ${AP_HAL_ROOT}/hwdef/scripts/chibios_hwdef.py -D ${BUILDROOT} ${AP_HAL_ROOT}/hwdef/${BOARD}/hwdef.dat',
        group='dynamic_sources',
        target=['hwdef.h', 'apj.prototype']
    )

    bld(
        # create the file modules/ChibiOS/include_dirs
        rule='touch Makefile && BUILDDIR=${BUILDDIR} CHIBIOS=${CH_ROOT} AP_HAL=${AP_HAL_ROOT} ${MAKE} pass -f ${AP_HAL_ROOT}/hwdef/${BOARD}/chibios_board.mk',
        group='dynamic_sources',
        target='modules/ChibiOS/include_dirs'
    )

    bld(
        # build libch.a from ChibiOS sources and hwdef.h
        rule="BUILDDIR='${BUILDDIR}' CHIBIOS='${CH_ROOT}' AP_HAL=${AP_HAL_ROOT} '${MAKE}' lib -f ${AP_HAL_ROOT}/hwdef/${BOARD}/chibios_board.mk",
        group='dynamic_sources',
        source=bld.bldnode.find_or_declare('hwdef.h'),
        target=['modules/ChibiOS/libch.a']
    )

    bld.env.LIB += ['ch']
    bld.env.LIBPATH += ['modules/ChibiOS/']
    wraplist = ['strerror_r']
    for w in wraplist:
        bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]

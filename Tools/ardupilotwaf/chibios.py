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
    print _dynamic_env_data['include_dirs']

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

class make_chibios_task(Task.Task):
    color = 'CYAN'
    always_run = True
    def run(self):
        build_dir = self.env.get_flat('BUILDDIR')
        ch_root = self.env.get_flat('CHIBIOS')
        make = self.env.get_flat('MAKE')
        ap_hal = self.env.get_flat('AP_HAL')
        brd_type = self.env.get_flat('BOARD') 
        return self.exec_command("BUILDDIR='{}' CHIBIOS='{}' AP_HAL={}\
                   '{}' lib -f {}/hwdef/{}/chibios_board.mk".format(
                   build_dir, ch_root, ap_hal, make, ap_hal, brd_type
                   ))
    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(make_chibios_task, self).exec_command(cmd, **kw)
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

@feature('ch_ap_program')
@after_method('process_source')
def chibios_firmware(self):
    self.link_task.always_run = True
    make_tsk = self.create_task('make_chibios_task',
                                group='dynamic_sources',
                                tgt=self.bld.bldnode.find_or_declare('modules/ChibiOS/libch.a'))
    make_tsk.env.BUILDDIR = self.bld.env.BUILDDIR
    make_tsk.env.CHIBIOS = self.bld.env.CH_ROOT
    make_tsk.env.AP_HAL = self.bld.env.AP_HAL_ROOT

    link_output = self.link_task.outputs[0]
    self.objcopy_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.apj').name)
    generate_fw_task = self.create_task('generate_fw',
                            src=link_output,
                            tgt=self.objcopy_target)
    generate_fw_task.set_run_after(self.link_task)
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
    env.SERIAL_PORT = srcpath('/dev/serial/by-id/*_STLink*')

def build(bld):
    bld(
        source='libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat' % bld.env.get_flat('BOARD'),
        rule='python ${AP_HAL_ROOT}/hwdef/scripts/chibios_hwdef.py -D ${BUILDROOT} ${AP_HAL_ROOT}/hwdef/${BOARD}/hwdef.dat',
        group='dynamic_sources',
        target=['hwdef.h', 'apj.prototype']
    )

    bld(
        rule='touch Makefile && BUILDDIR=${BUILDDIR} CHIBIOS=${CH_ROOT} AP_HAL=${AP_HAL_ROOT} ${MAKE} pass -f ${AP_HAL_ROOT}/hwdef/${BOARD}/chibios_board.mk',
        group='dynamic_sources',
        target='modules/ChibiOS/include_dirs'
    )

    bld.env.LIB += ['ch']
    bld.env.LIBPATH += ['modules/ChibiOS/']
    wraplist = ['strerror_r']
    for w in wraplist:
        bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]
    
# @feature('ch_ap_program')
# @after_method('process_source')
# def chibios_firmware(self):
#     fw_task = self.create_task('make_chibios', self.bld.bldnode.make_node(self.program_dir), self.bld.bldnode.make_node(self.program_name))
#     fw_task.set_run_after(self.link_task)
#     fw_task.env.env = dict(os.environ)
#     fw_task.env.env['BUILDDIR'] = self.bld.bldnode.find_or_declare('modules/ChibiOS').abspath()
#     fw_task.env.env['CH_ROOT'] = self.bld.srcnode.make_node('modules/ChibiOS').abspath()
#     fw_task.env['AP_HAL'] = self.bld.srcnode.make_node('libraries/AP_HAL_ChibiOS').abspath()

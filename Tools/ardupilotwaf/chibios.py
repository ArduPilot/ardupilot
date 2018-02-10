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
        return self.exec_command("python {}/px_uploader.py {}".format(upload_tools, src))

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
    --prototype ${BUILDROOT}/apj.prototype > ${TGT} && \
    ${TOOLS_SCRIPTS}/make_abin.sh ${SRC}.bin ${SRC}.abin'
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
    env.CHIBIOS_SCRIPTS = srcpath('libraries/AP_HAL_ChibiOS/hwdef/scripts')
    env.TOOLS_SCRIPTS = srcpath('Tools/scripts')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')
    env.SERIAL_PORT = srcpath('/dev/serial/by-id/*_STLink*')

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

    hwdef = srcpath('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat' % env.BOARD)
    hwdef_script = srcpath('libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    try:
        cmd = 'python %s -D %s %s' % (hwdef_script, hwdef_out, hwdef)
        ret = subprocess.call(cmd, shell=True)
    except Exception:
        print("Failed to generate hwdef.h")

def build(bld):
    bld(
        # build hwdef.h and apj.prototype from hwdef.dat. This is needed after a waf clean
        source=bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat' % bld.env.get_flat('BOARD')),
        rule='python ${AP_HAL_ROOT}/hwdef/scripts/chibios_hwdef.py -D ${BUILDROOT} ${AP_HAL_ROOT}/hwdef/${BOARD}/hwdef.dat',
        group='dynamic_sources',
        target=['hwdef.h', 'apj.prototype', 'ldscript.ld']
    )
    
    bld(
        # create the file modules/ChibiOS/include_dirs
        rule='touch Makefile && BUILDDIR=${BUILDDIR} CHIBIOS=${CH_ROOT} AP_HAL=${AP_HAL_ROOT} ${CHIBIOS_FATFS_FLAG} ${CHIBIOS_BOARD_NAME} ${MAKE} pass -f ${BOARD_MK}',
        group='dynamic_sources',
        target='modules/ChibiOS/include_dirs'
    )

    common_src = [bld.bldnode.find_or_declare('hwdef.h'),
                  bld.bldnode.find_or_declare('modules/ChibiOS/include_dirs')]
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.[ch]')
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.mk')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.[ch]')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.mk')
    ch_task = bld(
        # build libch.a from ChibiOS sources and hwdef.h
        rule="BUILDDIR='${BUILDDIR}' CHIBIOS='${CH_ROOT}' AP_HAL=${AP_HAL_ROOT} ${CHIBIOS_FATFS_FLAG} ${CHIBIOS_BOARD_NAME} '${MAKE}' lib -f ${BOARD_MK}",
        group='dynamic_sources',
        source=common_src,
        target='modules/ChibiOS/libch.a'
    )
    ch_task.name = "ChibiOS_lib"

    bld.env.LIB += ['ch']
    bld.env.LIBPATH += ['modules/ChibiOS/']
    wraplist = ['strerror_r', 'fclose', 'freopen', 'fread']
    for w in wraplist:
        bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]

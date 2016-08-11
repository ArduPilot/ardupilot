#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for VRUBRAIN build
"""

from waflib import Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os
import sys

_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/PX4Firmware')
    for name in ('cxx_flags', 'include_dirs', 'definitions'):
        _dynamic_env_data[name] = bldnode.find_node(name).read().split(';')

@feature('vrubrain_ap_stlib', 'vrubrain_ap_program')
@before_method('process_source')
def vrubrain_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    if not _dynamic_env_data:
        _load_dynamic_env_data(self.bld)

    self.env.append_value('INCLUDES', _dynamic_env_data['include_dirs'])
    self.env.prepend_value('CXXFLAGS', _dynamic_env_data['cxx_flags'])
    self.env.prepend_value('CXXFLAGS', _dynamic_env_data['definitions'])

# Single static library
# NOTE: This only works only for local static libraries dependencies - fake
# libraries aren't supported yet
@feature('vrubrain_ap_program')
@after_method('apply_link')
@before_method('process_use')
def vrubrain_import_objects_from_use(self):
    queue = Utils.to_list(getattr(self, 'use', []))
    names = set()

    while queue:
        name = queue.pop(0)
        if name in names:
            continue
        names.add(name)

        try:
            tg = self.bld.get_tgen_by_name(name)
        except Errors.WafError:
            continue

        tg.post()
        for t in getattr(tg, 'compiled_tasks', []):
            self.link_task.set_inputs(t.outputs)

        queue.extend(Utils.to_list(getattr(tg, 'use', [])))

class vrubrain_copy(Task.Task):
    run_str = '${CP} ${SRC} ${TGT}'
    color = 'CYAN'

    def keyword(self):
        return "VRUBRAIN: Copying %s to" % self.inputs[0].name

    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class vrubrain_add_git_hashes(Task.Task):
    run_str = '${PYTHON} ${PX4_ADD_GIT_HASHES} --ardupilot ${PX4_APM_ROOT} --px4 ${PX4_ROOT} --nuttx ${PX4_NUTTX_ROOT} --uavcan ${PX4_UAVCAN_ROOT} ${SRC} ${TGT}'
    color = 'CYAN'

    def keyword(self):
        return "VRUBRAIN: Copying firmware and adding git hashes"

    def __str__(self):
        return self.outputs[0].path_from(self.outputs[0].ctx.launch_node())

def _update_firmware_sig(fw_task, firmware, elf):
    original_post_run = fw_task.post_run
    def post_run():
        original_post_run()
        firmware.sig = firmware.cache_sig = Utils.h_file(firmware.abspath())
        elf.sig = elf.cache_sig = Utils.h_file(elf.abspath())
    fw_task.post_run = post_run

_firmware_semaphorish_tasks = []
_upload_task = []

@feature('vrubrain_ap_program')
@after_method('process_source')
def vrubrain_firmware(self):
    global _firmware_semaphorish_tasks, _upload_task
    version = self.env.get_flat('VRUBRAIN_VERSION')

    vrubrain = self.bld.cmake('vrubrain')
    vrubrain.vars['APM_PROGRAM_LIB'] = self.link_task.outputs[0].abspath()

    fw_task = self.create_cmake_build_task(
        'vrubrain',
        'build_firmware_vrubrain-v%s' % version,
    )

    # we need to synchronize in order to avoid the output expected by the
    # previous ap_program being overwritten before used
    for t in _firmware_semaphorish_tasks:
        fw_task.set_run_after(t)
    _firmware_semaphorish_tasks = []

    firmware = vrubrain.bldnode.make_node(
        'src/firmware/nuttx/nuttx-vrubrain-v%s-apm.px4' % version,
    )
    fw_elf = vrubrain.bldnode.make_node(
        'src/firmware/nuttx/firmware_nuttx',
    )
    _update_firmware_sig(fw_task, firmware, fw_elf)

    fw_dest = self.bld.bldnode.make_node(
        os.path.join(self.program_dir, '%s.px4' % self.program_name)
    )
    git_hashes = self.create_task('vrubrain_add_git_hashes', firmware, fw_dest)
    git_hashes.set_run_after(fw_task)
    _firmware_semaphorish_tasks.append(git_hashes)

    fw_elf_dest = self.bld.bldnode.make_node(
        os.path.join(self.program_dir, self.program_name)
    )
    cp_elf = self.create_task('vrubrain_copy', fw_elf, fw_elf_dest)
    cp_elf.set_run_after(fw_task)
    _firmware_semaphorish_tasks.append(cp_elf)

    self.build_summary = dict(
        target=self.name,
        binary=fw_elf_dest.path_from(self.bld.bldnode),
    )

    if self.bld.options.upload:
        if _upload_task:
            Logs.warn('VRUBRAIN: upload for %s ignored' % self.name)
            return
        _upload_task = self.create_cmake_build_task('vrubrain', 'upload')
        _upload_task.set_run_after(fw_task)
        _firmware_semaphorish_tasks.append(_upload_task)

def _vrubrain_taskgen(bld, **kw):
    if 'cls_keyword' in kw and not callable(kw['cls_keyword']):
        cls_keyword = str(kw['cls_keyword'])
        kw['cls_keyword'] = lambda tsk: 'VRUBRAIN: ' + cls_keyword

    if 'cls_str' in kw and not callable(kw['cls_str']):
        cls_str = str(kw['cls_str'])
        kw['cls_str'] = lambda tsk: cls_str

    kw['color'] = 'CYAN'

    return bld(**kw)

@feature('_vrubrain_romfs')
def _process_romfs(self):
    bld = self.bld
    file_list = (
        'init.d/rc.APM',
        'init.d/rc.error',
        'init.d/rcS',
    )

    romfs_src = bld.srcnode.find_dir(bld.env.PX4_ROMFS_SRC)
    romfs_bld = bld.bldnode.make_node(bld.env.PX4_ROMFS_BLD)

    for item in file_list:
        if isinstance(item, str):
            src = romfs_src.make_node(item)
            dst = romfs_bld.make_node(item)
        else:
            src = bld.srcnode.make_node(item[0])
            dst = romfs_bld.make_node(item[1])

        dst.parent.mkdir()
        self.create_task('vrubrain_copy', src, dst)

def configure(cfg):
    cfg.env.CMAKE_MIN_VERSION = '3.2'
    cfg.load('cmake')
    cfg.find_program('cp')

    bldnode = cfg.bldnode.make_node(cfg.variant)
    env = cfg.env

    env.AP_PROGRAM_FEATURES += ['vrubrain_ap_program']
    env.AP_STLIB_FEATURES += ['vrubrain_ap_stlib']

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()


    # TODO: we should move stuff from mk/VRBRAIN to Tools/ardupilotwaf/vrbrain after
    # stop using the make-based build system
    env.PX4_ROMFS_SRC = 'mk/VRBRAIN/ROMFS'
    env.PX4_ROMFS_BLD = 'vrbrain-extra-files/ROMFS'

    env.PX4_ADD_GIT_HASHES = srcpath('Tools/scripts/add_git_hashes.py')
    env.PX4_APM_ROOT = srcpath('')
    env.PX4_ROOT = srcpath('modules/PX4Firmware')
    env.PX4_NUTTX_ROOT = srcpath('modules/PX4NuttX')
    env.PX4_UAVCAN_ROOT = srcpath('modules/uavcan')

    env.PX4_CMAKE_VARS = dict(
        CONFIG='nuttx_vrubrain-v%s_apm' % env.get_flat('VRUBRAIN_VERSION'),
        CMAKE_MODULE_PATH=srcpath('Tools/ardupilotwaf/vrbrain/cmake'),
        UAVCAN_LIBUAVCAN_PATH=env.PX4_UAVCAN_ROOT,
        NUTTX_SRC=env.PX4_NUTTX_ROOT,
        PX4_NUTTX_ROMFS=bldpath(env.PX4_ROMFS_BLD),
        ARDUPILOT_BUILD='YES',
        EXTRA_CXX_FLAGS=' '.join((
            # NOTE: these "-Wno-error=*" flags should be removed as we update
            # the submodule
            '-Wno-error=double-promotion',
            '-Wno-error=reorder',
            # NOTE: *Temporarily* using this definition so that both
            # PX4Firmware build systems (cmake and legacy make-based) can live
            # together
            '-DCMAKE_BUILD',
            '-DARDUPILOT_BUILD',
            '-I%s' % bldpath('libraries/GCS_MAVLink'),
            '-I%s' % bldpath('libraries/GCS_MAVLink/include/mavlink'),
            '-Wl,--gc-sections',
        )),
        EXTRA_C_FLAGS=' '.join((
            # NOTE: *Temporarily* using this definition so that both
            # PX4Firmware build systems (cmake and legacy make-based) can live
            # together
            '-DCMAKE_BUILD',
        )),
    )

def build(bld):
    version = bld.env.get_flat('VRUBRAIN_VERSION')
    vrubrain = bld.cmake(
        name='vrubrain',
        cmake_src=bld.srcnode.find_dir('modules/PX4Firmware'),
        cmake_vars=bld.env.PX4_CMAKE_VARS,
    )

    vrubrain.build(
        'msg_gen',
        group='dynamic_sources',
        cmake_output_patterns='src/modules/uORB/topics/*.h',
    )
    vrubrain.build(
        'prebuild_targets',
        group='dynamic_sources',
        cmake_output_patterns='vrubrain-v%s/NuttX/nuttx-export/**/*.h' % version,
    )

    bld(
        name='vrubrain_romfs_static_files',
        group='dynamic_sources',
        features='_vrubrain_romfs',
    )

    bld.extra_build_summary = _extra_build_summary

def _extra_build_summary(bld, build_summary):
    build_summary.text('')
    build_summary.text('VRUBRAIN')
    build_summary.text('', '''
The ELF files are pointed by the path in the "%s" column. The .px4 files are in
the same directory of their corresponding ELF files.
''' % build_summary.header_text['target'])

    if not bld.options.upload:
        build_summary.text('')
        build_summary.text('', '''
You can use the option --upload to upload the firmware to the VRUBRAIN board if you
have one connected.''')


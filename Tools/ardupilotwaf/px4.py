#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for PX4 build
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

    _dynamic_env_data['DEFINES'] = [
        'NUTTX_GIT_VERSION="%s"' % bld.git_submodule_head_hash('PX4NuttX')[:8],
        'PX4_GIT_VERSION="%s"' % bld.git_submodule_head_hash('PX4Firmware')[:8],
    ]

@feature('px4_ap_stlib', 'px4_ap_program')
@before_method('process_source')
def px4_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    if not _dynamic_env_data:
        _load_dynamic_env_data(self.bld)

    self.env.append_value('INCLUDES', _dynamic_env_data['include_dirs'])
    self.env.prepend_value('CXXFLAGS', _dynamic_env_data['cxx_flags'])
    self.env.prepend_value('CXXFLAGS', _dynamic_env_data['definitions'])
    self.env.DEFINES += _dynamic_env_data['DEFINES']

# Single static library
# NOTE: This only works only for local static libraries dependencies - fake
# libraries aren't supported yet
@feature('px4_ap_program')
@after_method('apply_link')
@before_method('process_use')
def px4_import_objects_from_use(self):
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

class px4_copy_lib(Task.Task):
    run_str = '${CP} ${SRC} ${PX4_AP_PROGRAM_LIB}'
    color = 'CYAN'

    def runnable_status(self):
        status = super(px4_copy_lib, self).runnable_status()
        if status != Task.SKIP_ME:
            return status

        pseudo_target = self.get_pseudo_target()
        try:
            if pseudo_target.sig != self.inputs[0].sig:
                status = Task.RUN_ME
        except AttributeError:
            status = Task.RUN_ME

        return status

    def get_pseudo_target(self):
        if hasattr(self, 'pseudo_target'):
            return self.pseudo_target

        bldnode = self.generator.bld.bldnode
        self.pseudo_target = bldnode.make_node(self.env.PX4_AP_PROGRAM_LIB)
        return self.pseudo_target

    def keyword(self):
        return 'PX4: Copying program library'

    def post_run(self):
        super(px4_copy_lib, self).post_run()
        pseudo_target = self.get_pseudo_target()
        pseudo_target.sig = pseudo_target.cache_sig = self.inputs[0].sig

class px4_copy(Task.Task):
    run_str = '${CP} ${SRC} ${TGT}'
    color = 'CYAN'

    def keyword(self):
        return "PX4: Copying %s to" % self.inputs[0].name

    def __str__(self):
        return self.outputs[0].path_from(self.outputs[0].ctx.launch_node())

_firmware_semaphorish_tasks = []

class px4_add_git_hashes(Task.Task):
    run_str = '${PYTHON} ${PX4_ADD_GIT_HASHES} --ardupilot ${PX4_APM_ROOT} --px4 ${PX4_ROOT} --nuttx ${PX4_NUTTX_ROOT} --uavcan ${PX4_UAVCAN_ROOT} ${SRC} ${TGT}'
    color = 'CYAN'

    def keyword(self):
        return "PX4: Copying firmware and adding git hashes"

    def __str__(self):
        return self.outputs[0].path_from(self.outputs[0].ctx.launch_node())

_upload_task = []

@feature('px4_ap_program')
@after_method('process_source')
def px4_firmware(self):
    global _firmware_semaphorish_tasks, _upload_task
    version = self.env.get_flat('PX4_VERSION')

    romfs = self.bld.srcnode.make_node(self.env.PX4_ROMFS)

    romfs_bootloader = romfs.make_node('bootloader/fmu_bl.bin')
    bootloader = self.bld.srcnode.make_node(self.env.PX4_BOOTLOADER)
    cp_bl = self.create_task('px4_copy', bootloader, romfs_bootloader)

    fw_io_tg = self.bld.get_tgen_by_name('px4_fw_io')
    fw_io_tg.post()

    px4io = fw_io_tg.cmake_build_task.outputs[0]
    romfs_px4io = romfs.make_node('px4io/px4io.bin')
    cp_px4io = self.create_task('px4_copy', px4io, romfs_px4io)

    cp_lib = self.create_task('px4_copy_lib', self.link_task.outputs)
    # we need to synchronize because the path PX4_AP_PROGRAM_LIB is used by all
    # ap_programs
    if _firmware_semaphorish_tasks:
        for t in _firmware_semaphorish_tasks:
            cp_lib.set_run_after(t)

    fw_task = self.create_cmake_build_task(
        'px4',
        'build_firmware_px4fmu-v%s' % version,
    )
    firmware = fw_task.config_taskgen.cmake_bld.make_node(
        'src/firmware/nuttx/nuttx-px4fmu-v%s-apm.px4' % version,
    )
    fw_task.set_outputs(firmware)
    fw_task.set_run_after(cp_lib)
    fw_task.dep_nodes.extend([romfs_px4io, romfs_bootloader])

    path = os.path.join(self.program_group, '%s.px4' % self.program_name)
    fw_dest = self.bld.bldnode.make_node(path)
    git_hashes = self.create_task('px4_add_git_hashes', firmware, fw_dest)
    _firmware_semaphorish_tasks.append(git_hashes)

    if self.bld.options.upload:
        if _upload_task:
            Logs.warn('PX4: upload for %s ignored' % self.name)
            return
        _upload_task = self.create_cmake_build_task('px4', 'upload')
        _upload_task.cmd_kw = dict(stdout=sys.stdout)
        _upload_task.set_run_after(fw_task)
        _firmware_semaphorish_tasks.append(_upload_task)

def configure(cfg):
    cfg.load('cmake')
    cfg.find_program('cp')

    bldnode = cfg.bldnode.make_node(cfg.variant)
    env = cfg.env

    env.AP_PROGRAM_FEATURES += ['px4_ap_program']
    env.AP_STLIB_FEATURES += ['px4_ap_stlib']

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()

    if env.PX4_VERSION == '1':
        bootloader_name = 'px4fmu_bl.bin'
    else:
        bootloader_name = 'px4fmuv%s_bl.bin' % env.get_flat('PX4_VERSION')

    # TODO: we should move stuff from mk/PX4 to Tools/ardupilotwaf/px4 after
    # stop using the make-based build system
    env.PX4_ROMFS = 'mk/PX4/ROMFS'
    env.PX4_BOOTLOADER = 'mk/PX4/bootloader/%s' % bootloader_name

    bldnode.make_node('px4-extra-files').mkdir()
    program_lib_name = cfg.env.cxxstlib_PATTERN % 'ap_program'
    env.PX4_AP_PROGRAM_LIB = os.path.join('px4-extra-files', program_lib_name)

    env.PX4_ADD_GIT_HASHES = srcpath('Tools/scripts/add_git_hashes.py')
    env.PX4_APM_ROOT = srcpath('')
    env.PX4_ROOT = srcpath('modules/PX4Firmware')
    env.PX4_NUTTX_ROOT = srcpath('modules/PX4NuttX')
    env.PX4_UAVCAN_ROOT = srcpath('modules/uavcan')

    env.PX4_CMAKE_VARS = dict(
        CONFIG='nuttx_px4fmu-v%s_apm' % env.get_flat('PX4_VERSION'),
        CMAKE_MODULE_PATH=srcpath('Tools/ardupilotwaf/px4/cmake'),
        UAVCAN_LIBUAVCAN_PATH=env.PX4_UAVCAN_ROOT,
        NUTTX_SRC=env.PX4_NUTTX_ROOT,
        PX4_NUTTX_ROMFS=srcpath(env.PX4_ROMFS),
        APM_PROGRAM_LIB=bldpath(env.PX4_AP_PROGRAM_LIB),
        EXTRA_CXX_FLAGS=' '.join((
            # NOTE: these "-Wno-error=*" flags should be removed as we update
            # the submodule
            '-Wno-error=double-promotion',
            '-Wno-error=reorder',
            # The current CMake build of PX4Firmware defines this as 48.
            # However, that fails our build due to some static assertion on
            # map.hpp. We may need to update our uavcan submodule.
            '-UUAVCAN_MEM_POOL_BLOCK_SIZE',
            '-UUAVCAN_STM32_TIMER_NUMBER',
            '-UUAVCAN_PLATFORM',
            # NOTE: *Temporarily* using this definition so that both
            # PX4Firmware build systems (cmake and legacy make-based) can live
            # together
            '-DCMAKE_BUILD',
        )),
        EXTRA_C_FLAGS=' '.join((
            # NOTE: *Temporarily* using this definition so that both
            # PX4Firmware build systems (cmake and legacy make-based) can live
            # together
            '-DCMAKE_BUILD',
        )),
    )

def build(bld):
    version = bld.env.get_flat('PX4_VERSION')
    px4 = bld(
        features='cmake_configure',
        name='px4',
        cmake_src=bld.srcnode.find_dir('modules/PX4Firmware'),
        cmake_vars=bld.env.PX4_CMAKE_VARS,
        group='dynamic_sources',
    )

    px4.cmake_build(
        'msg_gen',
        group='dynamic_sources',
        cmake_output_patterns='src/modules/uORB/topics/*.h',
    )
    px4.cmake_build(
        'prebuild_targets',
        group='dynamic_sources',
        cmake_output_patterns='px4fmu-v%s/NuttX/nuttx-export/**/*.h' % version,
    )

    if bld.env.PX4_USE_PX4IO:
        px4.cmake_build(
            'fw_io',
            target=px4.get_cmake_bldnode().find_or_declare(
                'src/modules/px4iofirmware/px4io-v%s.bin' % version,
            ),
        )

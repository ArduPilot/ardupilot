#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for PX4 build
"""

from waflib import Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os

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

class px4_copy(Task.Task):
    run_str = '${CP} ${SRC} ${TGT}'
    color = 'CYAN'

    def keyword(self):
        return "PX4: Copying %s to" % self.inputs[0].name

    def __str__(self):
        return self.outputs[0].path_from(self.outputs[0].ctx.launch_node())

@feature('px4_ap_program')
@after_method('process_source')
def px4_firmware(self):
    romfs = self.bld.srcnode.make_node(self.env.PX4_ROMFS)

    romfs_bootloader = romfs.make_node('bootloader/fmu_bl.bin')
    bootloader = self.bld.srcnode.make_node(self.env.PX4_BOOTLOADER)
    cp_bl = self.create_task('px4_copy', bootloader, romfs_bootloader)

    fw_io_tg = self.bld.get_tgen_by_name('px4_fw_io')
    fw_io_tg.post()

    px4io = fw_io_tg.cmake_build_task.outputs[0]
    romfs_px4io = romfs.make_node('px4io/px4io.bin')
    cp_px4io = self.create_task('px4_copy', px4io, romfs_px4io)

def configure(cfg):
    cfg.load('cmake')
    cfg.find_program('cp')

    env = cfg.env

    env.AP_PROGRAM_FEATURES += ['px4_ap_program']
    env.AP_STLIB_FEATURES += ['px4_ap_stlib']

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    if env.PX4_VERSION == '1':
        bootloader_name = 'px4fmu_bl.bin'
    else:
        bootloader_name = 'px4fmuv%s_bl.bin' % env.get_flat('PX4_VERSION')
    # TODO: we should move stuff from mk/PX4 to Tools/ardupilotwaf/px4 after
    # stop using the make-based build system
    env.PX4_ROMFS = 'mk/PX4/ROMFS'
    env.PX4_BOOTLOADER = 'mk/PX4/bootloader/%s' % bootloader_name

    env.PX4_CMAKE_VARS = dict(
        CONFIG='nuttx_px4fmu-v%s_apm' % env.get_flat('PX4_VERSION'),
        CMAKE_MODULE_PATH=srcpath('Tools/ardupilotwaf/px4/cmake'),
        UAVCAN_LIBUAVCAN_PATH=srcpath('modules/uavcan'),
        NUTTX_SRC=srcpath('modules/PX4NuttX'),
        PX4_NUTTX_ROMFS=srcpath(env.PX4_ROMFS),
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

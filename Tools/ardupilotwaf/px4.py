#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for PX4 build
"""

import os

def configure(cfg):
    cfg.load('cmake')

    env = cfg.env

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    env.PX4_CMAKE_VARS = dict(
        CONFIG='nuttx_px4fmu-v%s_apm' % env.get_flat('PX4_VERSION'),
        CMAKE_MODULE_PATH=srcpath('Tools/ardupilotwaf/px4/cmake'),
        UAVCAN_LIBUAVCAN_PATH=srcpath('modules/uavcan'),
        NUTTX_SRC=srcpath('modules/PX4NuttX'),
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

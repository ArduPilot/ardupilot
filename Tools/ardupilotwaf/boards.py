#!/usr/bin/env python
# encoding: utf-8

from collections import OrderedDict
import sys

import waflib
from waflib.Configure import conf

_board_classes = {}

class BoardMeta(type):
    def __init__(cls, name, bases, dct):
        super(BoardMeta, cls).__init__(name, bases, dct)

        if 'abstract' not in cls.__dict__:
            cls.abstract = False
        if cls.abstract:
            return

        if not hasattr(cls, 'toolchain'):
            cls.toolchain = 'native'

        board_name = getattr(cls, 'name', name)
        if board_name in _board_classes:
            raise Exception('board named %s already exists' % board_name)
        _board_classes[board_name] = cls

class Board:
    abstract = True

    def __init__(self):
        self.with_uavcan = False

    def configure(self, cfg):
        cfg.env.TOOLCHAIN = self.toolchain
        cfg.load('toolchain')
        cfg.load('cxx_checks')

        env = waflib.ConfigSet.ConfigSet()
        self.configure_env(cfg, env)

        d = env.get_merged_dict()
        # Always prepend so that arguments passed in the command line get
        # the priority.
        for k, val in d.items():
            # Dictionaries (like 'DEFINES') are converted to lists to
            # conform to waf conventions.
            if isinstance(val, dict):
                keys = list(val.keys())
                if not isinstance(val, OrderedDict):
                    keys.sort()
                val = ['%s=%s' % (vk, val[vk]) for vk in keys]

            if k in cfg.env and isinstance(cfg.env[k], list):
                cfg.env.prepend_value(k, val)
            else:
                cfg.env[k] = val

        cfg.ap_common_checks()

        cfg.env.prepend_value('INCLUDES', [
            cfg.srcnode.find_dir('libraries/AP_Common/missing').abspath()
        ])


    def configure_env(self, cfg, env):
        # Use a dictionary instead of the convetional list for definitions to
        # make easy to override them. Convert back to list before consumption.
        env.DEFINES = {}

        env.CFLAGS += [
            '-ffunction-sections',
            '-fdata-sections',
            '-fsigned-char',

            '-Wall',
            '-Wextra',
            '-Wformat',
            '-Wshadow',
            '-Wpointer-arith',
            '-Wcast-align',
            '-Wundef',
            '-Wno-missing-field-initializers',
            '-Wno-unused-parameter',
            '-Wno-redundant-decls',
            '-Wno-unknown-pragmas',
        ]

        if 'clang' in cfg.env.COMPILER_CC:
            env.CFLAGS += [
                '-fcolor-diagnostics',

                '-Wno-gnu-designator',
                '-Wno-inconsistent-missing-override',
                '-Wno-mismatched-tags',
                '-Wno-gnu-variable-sized-type-not-at-end',
                '-Wno-c++11-narrowing'
            ]

        if cfg.env.DEBUG:
            env.CFLAGS += [
                '-g',
                '-O0',
            ]

        env.CXXFLAGS += [
            '-std=gnu++11',

            '-fdata-sections',
            '-ffunction-sections',
            '-fno-exceptions',
            '-fsigned-char',

            '-Wall',
            '-Wextra',
            '-Wformat',
            '-Wshadow',
            '-Wpointer-arith',
            '-Wcast-align',
            '-Wundef',
            '-Wno-unused-parameter',
            '-Wno-missing-field-initializers',
            '-Wno-reorder',
            '-Wno-redundant-decls',
            '-Wno-unknown-pragmas',
            '-Werror=format-security',
            '-Werror=array-bounds',
            '-Werror=uninitialized',
            '-Werror=init-self',
            '-Wfatal-errors',
        ]

        if 'clang++' in cfg.env.COMPILER_CXX:
            env.CXXFLAGS += [
                '-fcolor-diagnostics',

                '-Wno-gnu-designator',
                '-Wno-inconsistent-missing-override',
                '-Wno-mismatched-tags',
                '-Wno-gnu-variable-sized-type-not-at-end',
                '-Wno-c++11-narrowing'
            ]
        else:
            env.CXXFLAGS += [
                '-Werror=unused-but-set-variable'
            ]

        if cfg.env.DEBUG:
            env.CXXFLAGS += [
                '-g',
                '-O0',
            ]

        if cfg.env.DEST_OS == 'darwin':
            env.LINKFLAGS += [
                '-Wl,-dead_strip',
            ]
        else:
            env.LINKFLAGS += [
                '-Wl,--gc-sections',
            ]

        if self.with_uavcan:
            env.AP_LIBRARIES += [
                'AP_UAVCAN',
                'modules/uavcan/libuavcan/src/**/*.cpp'
                ]

            env.CXXFLAGS += [
                '-Wno-error=cast-align',
            ]
            
            env.DEFINES.update(
                UAVCAN_CPP_VERSION = 'UAVCAN_CPP03',
                UAVCAN_NO_ASSERTIONS = 1,
                UAVCAN_NULLPTR = 'nullptr'
            )

            env.INCLUDES += [
                cfg.srcnode.find_dir('modules/uavcan/libuavcan/include').abspath()
            ]

        # We always want to use PRI format macros
        cfg.define('__STDC_FORMAT_MACROS', 1)


    def build(self, bld):
        bld.ap_version_append_str('GIT_VERSION', bld.git_head_hash(short=True))

Board = BoardMeta('Board', Board.__bases__, dict(Board.__dict__))

def get_boards_names():
    return sorted(list(_board_classes.keys()))

_board = None
@conf
def get_board(ctx):
    global _board
    if not _board:
        if not ctx.env.BOARD:
            ctx.fatal('BOARD environment variable must be set before first call to get_board()')
        _board = _board_classes[ctx.env.BOARD]()
    return _board

# NOTE: Keeping all the board definitions together so we can easily
# identify opportunities to simplify common flags. In the future might
# be worthy to keep board definitions in files of their own.

class sitl(Board):
    def configure_env(self, cfg, env):
        super(sitl, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_SITL',
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_NONE',
        )

        if not cfg.env.DEBUG:
            env.CXXFLAGS += [
                '-O3',
            ]

        env.LIB += [
            'm',
        ]

        cfg.check_librt(env)

        env.LINKFLAGS += ['-pthread',]
        env.AP_LIBRARIES += [
            'AP_HAL_SITL',
            'SITL',
        ]

        if sys.platform == 'cygwin':
            env.LIB += [
                'winmm',
            ]

class linux(Board):
    def configure_env(self, cfg, env):
        super(linux, self).configure_env(cfg, env)

        cfg.find_toolchain_program('pkg-config', var='PKGCONFIG')

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_LINUX',
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_NONE',
        )

        if not cfg.env.DEBUG:
            env.CXXFLAGS += [
                '-O3',
            ]

        env.LIB += [
            'm',
        ]

        cfg.check_librt(env)
        cfg.check_lttng(env)
        cfg.check_libdl(env)
        cfg.check_libiio(env)

        env.LINKFLAGS += ['-pthread',]
        env.AP_LIBRARIES = [
            'AP_HAL_Linux',
        ]


class minlure(linux):
    def configure_env(self, cfg, env):
        super(minlure, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_MINLURE',
        )


class erleboard(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(erleboard, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD',
        )

class navio(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(navio, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_NAVIO',
        )

class navio2(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(navio2, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_NAVIO2',
        )

class zynq(linux):
    toolchain = 'arm-xilinx-linux-gnueabi'

    def configure_env(self, cfg, env):
        super(zynq, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ZYNQ',
        )

class bbbmini(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(bbbmini, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BBBMINI',
        )

class blue(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(blue, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BLUE',
        )

class pxf(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(pxf, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_PXF',
        )

class bebop(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(bebop, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BEBOP',
        )

class disco(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(disco, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_DISCO',
        )

class raspilot(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(raspilot, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_RASPILOT',
        )

class erlebrain2(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(erlebrain2, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2',
        )

class bhat(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(bhat, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BH',
        )

class dark(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(dark, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_DARK',
        )

class urus(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(urus, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_URUS',
        )

class pxfmini(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(pxfmini, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_PXFMINI',
        )

class aero(linux):
    def configure_env(self, cfg, env):
        super(aero, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_AERO',
        )

class px4(Board):
    abstract = True
    toolchain = 'arm-none-eabi'

    def __init__(self):
        # bootloader name: a file with that name will be used and installed
        # on ROMFS
        super(px4, self).__init__()

        self.bootloader_name = None

        # board name: it's the name of this board that's also used as path
        # in ROMFS: don't add spaces
        self.board_name = None

        # px4io binary name: this is the name of the IO binary to be installed
        # in ROMFS
        self.px4io_name = None

        # board-specific init script: if True a file with `board_name` name will
        # be searched for in sources and installed in ROMFS as rc.board. This
        # init script is used to change the init behavior among different boards.
        self.board_rc = False
        self.ROMFS_EXCLUDE = []

    def configure(self, cfg):
        if not self.bootloader_name:
            cfg.fatal('configure: px4: bootloader name is required')
        if not self.board_name:
            cfg.fatal('configure: px4: board name is required')

        super(px4, self).configure(cfg)
        cfg.load('px4')

    def configure_env(self, cfg, env):
        super(px4, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_PX4',
            HAVE_OCLOEXEC = 0,
            HAVE_STD_NULLPTR_T = 0,
        )
        env.CXXFLAGS += [
            '-Wlogical-op',
            '-Wframe-larger-than=1300',
            '-fsingle-precision-constant',
            '-Wno-error=double-promotion',
            '-Wno-error=missing-declarations',
            '-Wno-error=float-equal',
            '-Wno-error=undef',
            '-Wno-error=cpp',
        ]
        env.AP_LIBRARIES += [
            'AP_HAL_PX4',
        ]
        env.GIT_SUBMODULES += [
            'PX4Firmware',
            'PX4NuttX',
            'uavcan',
        ]

        env.ROMFS_EXCLUDE = self.ROMFS_EXCLUDE

        env.PX4_BOOTLOADER_NAME = self.bootloader_name
        env.PX4_BOARD_NAME = self.board_name
        env.PX4_BOARD_RC = self.board_rc
        env.PX4_PX4IO_NAME = self.px4io_name

        env.AP_PROGRAM_AS_STLIB = True

    def build(self, bld):
        super(px4, self).build(bld)
        bld.ap_version_append_str('NUTTX_GIT_VERSION', bld.git_submodule_head_hash('PX4NuttX', short=True))
        bld.ap_version_append_str('PX4_GIT_VERSION', bld.git_submodule_head_hash('PX4Firmware', short=True))
        bld.load('px4')

    def romfs_exclude(self, exclude):
        self.ROMFS_EXCLUDE += exclude

class px4_v1(px4):
    name = 'px4-v1'
    def __init__(self):
        super(px4_v1, self).__init__()
        self.bootloader_name = 'px4fmu_bl.bin'
        self.board_name = 'px4fmu-v1'
        self.px4io_name = 'px4io-v1'
        self.romfs_exclude(['oreoled.bin'])

class px4_v2(px4):
    name = 'px4-v2'
    def __init__(self):
        super(px4_v2, self).__init__()
        self.bootloader_name = 'px4fmuv2_bl.bin'
        self.board_name = 'px4fmu-v2'
        self.px4io_name = 'px4io-v2'
        self.romfs_exclude(['oreoled.bin'])
        self.with_uavcan = True

class px4_v3(px4):
    name = 'px4-v3'
    def __init__(self):
        super(px4_v3, self).__init__()
        self.bootloader_name = 'px4fmuv2_bl.bin'
        self.board_name = 'px4fmu-v3'
        self.px4io_name = 'px4io-v2'
        self.with_uavcan = True

class px4_v4(px4):
    name = 'px4-v4'
    def __init__(self):
        super(px4_v4, self).__init__()
        self.bootloader_name = 'px4fmuv4_bl.bin'
        self.board_name = 'px4fmu-v4'
        self.romfs_exclude(['oreoled.bin'])
        self.with_uavcan = True

class aerofc_v1(px4):
    name = 'aerofc-v1'
    def __init__(self):
        super(aerofc_v1, self).__init__()
        self.bootloader_name = 'aerofcv1_bl.bin'
        self.board_name = 'aerofc-v1'
        self.romfs_exclude(['oreoled.bin'])
        self.board_rc = True

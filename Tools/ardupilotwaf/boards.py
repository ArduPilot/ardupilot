#!/usr/bin/env python
# encoding: utf-8

from collections import OrderedDict
import sys

import waflib

_board_classes = {}

class BoardMeta(type):
    def __init__(cls, name, bases, dct):
        super(BoardMeta, cls).__init__(name, bases, dct)

        if 'abstract' not in cls.__dict__:
            cls.abstract = False
        if cls.abstract:
            return

        board_name = getattr(cls, 'name', name)
        if board_name in _board_classes:
            raise Exception('board named %s already exists' % board_name)
        _board_classes[board_name] = cls

class Board:
    abstract = True

    def configure(self, cfg):
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

            if k in cfg.env and isinstance(cfg.env, list):
                cfg.env.prepend_value(k, val)
            else:
                cfg.env[k] = val

        cfg.load('toolchain')
        cfg.load('compiler_cxx compiler_c')

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
            '-Werror=format-security',
            '-Werror=array-bounds',
            '-Werror=unused-but-set-variable',
            '-Werror=uninitialized',
            '-Werror=init-self',
            '-Wfatal-errors',
        ]

        env.LINKFLAGS += [
            '-Wl,--gc-sections',
        ]

    def build(self, bld):
        pass

Board = BoardMeta('Board', Board.__bases__, dict(Board.__dict__))

def get_boards_names():
    return sorted(list(_board_classes.keys()))

_board = None
def get_board(name):
    global _board
    if not _board:
        _board = _board_classes[name]()
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
        env.CXXFLAGS += [
            '-O3',
        ]
        env.LIB += [
            'm',
        ]
        env.LINKFLAGS += ['-pthread',]
        env.AP_LIBRARIES += [
            'AP_HAL_SITL',
            'SITL',
        ]

class linux(Board):
    def configure_env(self, cfg, env):
        super(linux, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_LINUX',
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_NONE',
        )
        env.CXXFLAGS += [
            '-O3',
        ]
        env.LIB += [
            'm',
            'rt',
        ]
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
    def configure_env(self, cfg, env):
        super(erleboard, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD',
        )

class navio(linux):
    def configure_env(self, cfg, env):
        super(navio, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_NAVIO',
        )

class navio2(linux):
    def configure_env(self, cfg, env):
        super(navio2, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_NAVIO2',
        )

class zynq(linux):
    def configure_env(self, cfg, env):
        super(zynq, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-xilinx-linux-gnueabi'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ZYNQ',
        )

class bbbmini(linux):
    def configure_env(self, cfg, env):
        super(bbbmini, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BBBMINI',
        )

class pxf(linux):
    def configure_env(self, cfg, env):
        super(pxf, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_PXF',
        )

class bebop(linux):
    def configure_env(self, cfg, env):
        super(bebop, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BEBOP',
        )
        env.STATIC_LINKING = True

class raspilot(linux):
    def configure_env(self, cfg, env):
        super(raspilot, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_RASPILOT',
        )

class erlebrain2(linux):
    def configure_env(self, cfg, env):
        super(erlebrain2, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2',
        )

class bhat(linux):
    def configure_env(self, cfg, env):
        super(bhat, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BH',
        )

class pxfmini(linux):
    def configure_env(self, cfg, env):
        super(pxfmini, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_PXFMINI',
        )

class px4(Board):
    abstract = True

    def __init__(self):
        self.version = None
        self.use_px4io = True

    def configure(self, cfg):
        if not self.version:
            cfg.fatal('configure: px4: version required')

        super(px4, self).configure(cfg)
        cfg.load('px4')

    def configure_env(self, cfg, env):
        super(px4, self).configure_env(cfg, env)

        env.TOOLCHAIN = 'arm-none-eabi'
        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_PX4',
            HAVE_STD_NULLPTR_T = 0,
        )
        env.prepend_value('INCLUDES', [
            cfg.srcnode.find_dir('libraries/AP_Common/missing').abspath()
        ])
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

        env.PX4_VERSION = self.version
        env.PX4_USE_PX4IO = True if self.use_px4io else False

        env.AP_PROGRAM_AS_STLIB = True

    def build(self, bld):
        super(px4, self).build(bld)
        bld.load('px4')

class px4_v1(px4):
    name = 'px4-v1'
    def __init__(self):
        super(px4_v1, self).__init__()
        self.version = '1'

class px4_v2(px4):
    name = 'px4-v2'
    def __init__(self):
        super(px4_v2, self).__init__()
        self.version = '2'

class px4_v4(px4):
    name = 'px4-v4'
    def __init__(self):
        super(px4_v4, self).__init__()
        self.version = '4'
        self.use_px4io = False

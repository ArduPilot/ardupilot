#!/usr/bin/env python
# encoding: utf-8

import sys

import waflib

_board_classes = {}

class BoardMeta(type):
    def __init__(cls, name, bases, dct):
        super(BoardMeta, cls).__init__(name, bases, dct)
        if name == 'Board':
            return
        _board_classes[name] = cls

class Board:
    def configure(self, cfg):
        env = waflib.ConfigSet.ConfigSet()
        self.configure_env(env)

        d = env.get_merged_dict()
        # Always prepend so that arguments passed in the command line get
        # the priority.
        for k, val in d.items():
            # Dictionaries (like 'DEFINES') are converted to lists to
            # conform to waf conventions.
            if isinstance(val, dict):
                for item in val.items():
                    cfg.env.prepend_value(k, '%s=%s' % item)
            else:
                cfg.env.prepend_value(k, val)

    def configure_env(self, env):
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

Board = BoardMeta('Board', Board.__bases__, dict(Board.__dict__))

def get_boards_names():
    return sorted(list(_board_classes.keys()))

def get_board(name):
    return _board_classes[name]()

# NOTE: Keeping all the board definitions together so we can easily
# identify opportunities to simplify common flags. In the future might
# be worthy to keep board definitions in files of their own.

class sitl(Board):
    def configure_env(self, env):
        super(sitl, self).configure_env(env)

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
    def configure_env(self, env):
        super(linux, self).configure_env(env)

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
    def configure_env(self, env):
        super(minlure, self).configure_env(env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_MINLURE',
        )


class erleboard(linux):
    def configure_env(self, env):
        super(erleboard, self).configure_env(env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD',
        )

class navio(linux):
    def configure_env(self, env):
        super(navio, self).configure_env(env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_NAVIO',
        )

class zynq(linux):
    def configure_env(self, env):
        super(zynq, self).configure_env(env)

        env.TOOLCHAIN = 'arm-xilinx-linux-gnueabi'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ZYNQ',
        )

class bbbmini(linux):
    def configure_env(self, env):
        super(bbbmini, self).configure_env(env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BBBMINI',
        )

class pxf(linux):
    def configure_env(self, env):
        super(pxf, self).configure_env(env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_PXF',
        )

class bebop(linux):
    def configure_env(self, env):
        super(bebop, self).configure_env(env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BEBOP',
        )
        env.STATIC_LINKING = [True]

class raspilot(linux):
    def configure_env(self, env):
        super(raspilot, self).configure_env(env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_RASPILOT',
        )

class erlebrain2(linux):
    def configure_env(self, env):
        super(erlebrain2, self).configure_env(env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2',
        )

class bhat(linux):
    def configure_env(self, env):
        super(bhat, self).configure_env(env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BH',
        )

class pxfmini(linux):
    def configure_env(self, env):
        super(pxfmini, self).configure_env(env)

        env.TOOLCHAIN = 'arm-linux-gnueabihf'
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_PXFMINI',
        )

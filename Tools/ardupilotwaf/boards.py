#!/usr/bin/env python
# encoding: utf-8

import sys

import waflib

BOARDS = {}

PROJECT_ENV = waflib.ConfigSet.ConfigSet()

def define_board(func, name, parent_name=None):
    if parent_name is None:
        parent = PROJECT_ENV
    elif parent_name not in BOARDS:
        print("Undefined parent board '%s' for '%s'" % (parent_name, name))
        sys.exit(1)
    else:
        parent = BOARDS[parent_name]

    env = parent.derive().detach()
    if name in BOARDS:
        print("Multiple definitions of " + name)
        sys.exit(1)
    BOARDS[name] = env
    func(env)

def get_boards_names():
    return sorted(list(BOARDS.keys()))

# Use a dictionary instead of the convetional list for definitions to
# make easy to override them. Convert back to list before consumption.
PROJECT_ENV.DEFINES = {}

PROJECT_ENV.CFLAGS += [
    '-ffunction-sections',
    '-fdata-sections',
    '-fsigned-char',

    '-Wformat',
    '-Wall',
    '-Wshadow',
    '-Wpointer-arith',
    '-Wcast-align',
    '-Wno-unused-parameter',
    '-Wno-missing-field-initializers',
]

PROJECT_ENV.CXXFLAGS += [
    '-std=gnu++11',

    '-fdata-sections',
    '-ffunction-sections',
    '-fno-exceptions',
    '-fsigned-char',

    '-Wformat',
    '-Wall',
    '-Wshadow',
    '-Wpointer-arith',
    '-Wcast-align',
    '-Wno-unused-parameter',
    '-Wno-missing-field-initializers',
    '-Wno-reorder',
    '-Werror=format-security',
    '-Werror=array-bounds',
    '-Wfatal-errors',
    '-Werror=unused-but-set-variable',
    '-Werror=uninitialized',
    '-Werror=init-self',
    '-Wno-missing-field-initializers',
]

PROJECT_ENV.LINKFLAGS += [
    '-Wl,--gc-sections',
]

# NOTE: Keeping all the board definitions together so we can easily
# identify opportunities to simplify common flags. In the future might
# be worthy to keep board definitions in files of their own.

def sitl(env):
    env.DEFINES.update(
        CONFIG_HAL_BOARD = 'HAL_BOARD_SITL',
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_NONE',
    )

    env.CXXFLAGS += [
        '-O3',
    ]

    env.LIB += [
        'm',
        'pthread',
    ]

    env.AP_LIBRARIES += [
        'AP_HAL_SITL',
        'SITL',
    ]

define_board(sitl, 'sitl')


def linux(env):
    env.DEFINES.update(
        CONFIG_HAL_BOARD = 'HAL_BOARD_LINUX',
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_NONE',
    )

    env.CXXFLAGS += [
        '-O3',
    ]

    env.LIB += [
        'm',
        'pthread',
        'rt',
    ]

    env.AP_LIBRARIES = [
        'AP_HAL_Linux',
    ]

define_board(linux, 'linux')


def minlure(env):
    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_MINLURE',
    )

define_board(minlure, 'minlure', 'linux')


def erleboard(env):
    env.TOOLCHAIN = 'arm-linux-gnueabihf'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD',
    )

define_board(erleboard, 'erleboard', 'linux')


def navio(env):
    env.TOOLCHAIN = 'arm-linux-gnueabihf'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_NAVIO',
    )

define_board(navio, 'navio', 'linux')


def zynq(env):
    env.TOOLCHAIN = 'arm-xilinx-linux-gnueabi'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ZYNQ',
    )

define_board(zynq, 'zynq', 'linux')


def bbbmini(env):
    env.TOOLCHAIN = 'arm-linux-gnueabihf'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BBBMINI',
    )

define_board(bbbmini, 'bbbmini', 'linux')


def pxf(env):
    env.TOOLCHAIN = 'arm-linux-gnueabihf'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_PXF',
    )

define_board(pxf, 'pxf', 'linux')


def bebop(env):
    env.TOOLCHAIN = 'arm-linux-gnueabihf'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BEBOP',
    )

    env.STATIC_LINKING = [True]

define_board(bebop, 'bebop', 'linux')


def raspilot(env):
    env.TOOLCHAIN = 'arm-linux-gnueabihf'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_RASPILOT',
    )

define_board(raspilot, 'raspilot', 'linux')


def erlebrain2(env):
    env.TOOLCHAIN = 'arm-linux-gnueabihf'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2',
    )

define_board(erlebrain2, 'erlebrain2', 'linux')


def bhat(env):
    env.TOOLCHAIN = 'arm-linux-gnueabihf'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BH',
    )

define_board(bhat, 'bhat', 'linux')


def pxfmini(env):
    env.TOOLCHAIN = 'arm-linux-gnueabihf'

    env.DEFINES.update(
        CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_PXFMINI',
    )

define_board(pxfmini, 'pxfmini', 'linux')

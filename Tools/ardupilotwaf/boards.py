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

def get_boards_names():
    return sorted(list(BOARDS.keys()))

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
			AP_HAL_BOARD_DRIVER = 'AP_HAL_SITL',
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

class avr(Board):

    abstract = True
    toolchain = 'avr'

    def configure_env(self, cfg, env):
        super(avr, self).configure_env(cfg, env)

        env.DEFINES.update(
            AP_AHRS_NAVEKF_AVAILABLE = 0,
            F_CPU = 16000000,
            HAL_OS_POSIX_IO = 0,
			HAL_OS_SOCKETS = 0,
        )

        if not cfg.env.DEBUG:
            env.CXXFLAGS += [
                '-O3',
            ]

        cfg.env.CFLAGS = [
            '-std=gnu++11',
            '-Os',
            '-ffunction-sections',
            '-fdata-sections',
            '-fsigned-char',
            '-fno-use-cxa-atexit',
            '-fno-exceptions',
            '-fpermissive',

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
            '-Wformat=2',
            '-Wl, --gc-sections, --relax, --debug-relax',
            '-Wno-error=double-promotion',
            '-Wno-error=reorder',
			'-Wno-error=format-security',
			'-Wno-format-nonliteral',
        ]

        cfg.env.CXXFLAGS = [
            '-std=gnu++11',
            '-Os',
            '-ffunction-sections',
            '-fdata-sections',
            '-fno-exceptions',
            '-fsigned-char',
            '-fno-use-cxa-atexit',
            '-fpermissive',

            '-Wformat',
            '-Wall',
            '-Wshadow',
            '-Wpointer-arith',
            '-Wcast-align',
            '-Wwrite-strings',
            '-Wformat=2',
            '-Wno-unused-parameter',
            '-Wno-missing-field-initializers',
            '-Wno-reorder',
            '-mcall-prologues',
            '-Wl, --gc-sections, --relax, --debug-relax',
            '-Wno-error=double-promotion',
            '-Wno-error=reorder',
            '-fno-threadsafe-statics',
			'-Wno-error=format-security',
			'-Wno-format-nonliteral',
        ]

        env.LIB += [
            'm',
        ]

        env.AP_LIBRARIES += [
            'AP_HAL',
            'AP_HAL_AVR',
        ]

        env.AP_PROGRAM_AS_STLIB = True

    def build(self, bld):
        bld.env.SIZE = 0
        super(avr, self).build(bld)

class apm2(avr):
    def configure_env(self, cfg, env):
        super(apm2, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_APM2',
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_AVR_APM2',
            AP_HAL_BOARD_DRIVER = 'AP_HAL_AVR_APM2',
        )

        cfg.env.CFLAGS += [
            '-mmcu=atmega2560',
        ]

        cfg.env.CXXFLAGS += [
            '-mmcu=atmega2560',
        ]

class apm1(avr):
    def configure_env(self, cfg, env):
        super(apm1, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_APM1',
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_AVR_APM1',
            AP_HAL_BOARD_DRIVER = 'AP_HAL_AVR_APM1',
        )

        cfg.env.CFLAGS += [
            '-mmcu=atmega1280',
        ]

        cfg.env.CXXFLAGS += [
            '-mmcu=atmega1280',
        ]

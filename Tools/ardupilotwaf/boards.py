#!/usr/bin/env python
# encoding: utf-8

from collections import OrderedDict
import sys, os
import fnmatch

import waflib
from waflib import Utils
from waflib.Configure import conf

_board_classes = {}
_board = None

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
        self.with_can = False

    def configure(self, cfg):
        cfg.env.TOOLCHAIN = cfg.options.toolchain or self.toolchain
        cfg.env.ROMFS_FILES = []
        cfg.load('toolchain')
        cfg.load('cxx_checks')

        env = waflib.ConfigSet.ConfigSet()
        self.configure_env(cfg, env)

        # Setup scripting, had to defer this to allow checking board size
        if ((not cfg.options.disable_scripting) and
            (not cfg.env.DISABLE_SCRIPTING) and
            ((cfg.env.BOARD_FLASH_SIZE is None) or
             (cfg.env.BOARD_FLASH_SIZE == []) or
             (cfg.env.BOARD_FLASH_SIZE > 1024))):

            env.DEFINES.update(
                ENABLE_SCRIPTING = 1,
                LUA_32BITS = 1,
                )

            env.AP_LIBRARIES += [
                'AP_Scripting',
                'AP_Scripting/lua/src',
                ]

        else:
            cfg.options.disable_scripting = True

        # allow GCS disable for AP_DAL example
        if cfg.options.no_gcs:
            env.CXXFLAGS += ['-DHAL_NO_GCS=1']
            
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

    def cc_version_gte(self, cfg, want_major, want_minor):
        (major, minor, patchlevel) = cfg.env.CC_VERSION
        return (int(major) > want_major or
                (int(major) == want_major and int(minor) >= want_minor))

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
            '-Werror=format',
            '-Wpointer-arith',
            '-Wcast-align',
            '-Wno-missing-field-initializers',
            '-Wno-unused-parameter',
            '-Wno-redundant-decls',
            '-Wno-unknown-pragmas',
            '-Wno-trigraphs',
            '-Werror=shadow',
            '-Werror=return-type',
            '-Werror=unused-result',
            '-Werror=unused-variable',
            '-Werror=narrowing',
            '-Werror=attributes',
            '-Werror=overflow',
            '-Werror=parentheses',
            '-Werror=format-extra-args',
            '-Werror=ignored-qualifiers',
            '-Werror=undef',
            '-DARDUPILOT_BUILD',
        ]

        if cfg.options.scripting_checks:
            env.DEFINES.update(
                AP_SCRIPTING_CHECKS = 1,
                )

        cfg.msg("CXX Compiler", "%s %s"  % (cfg.env.COMPILER_CXX, ".".join(cfg.env.CC_VERSION)))

        if 'clang' in cfg.env.COMPILER_CC:
            env.CFLAGS += [
                '-fcolor-diagnostics',

                '-Wno-gnu-designator',
                '-Wno-inconsistent-missing-override',
                '-Wno-mismatched-tags',
                '-Wno-gnu-variable-sized-type-not-at-end',
                '-Werror=implicit-fallthrough',
            ]
        else:
            env.CFLAGS += [
                '-Wno-format-contains-nul',
            ]
            if self.cc_version_gte(cfg, 7, 4):
                env.CXXFLAGS += [
                    '-Werror=implicit-fallthrough',
                ]

        if cfg.env.DEBUG:
            env.CFLAGS += [
                '-g',
                '-O0',
            ]
            env.DEFINES.update(
                HAL_DEBUG_BUILD = 1,
            )

        if cfg.options.bootloader:
            # don't let bootloaders try and pull scripting in
            cfg.options.disable_scripting = True
        else:
            env.DEFINES.update(
                ENABLE_HEAP = 1,
            )

        if cfg.options.enable_math_check_indexes:
            env.CXXFLAGS += ['-DMATH_CHECK_INDEXES']

        env.CXXFLAGS += [
            '-std=gnu++11',

            '-fdata-sections',
            '-ffunction-sections',
            '-fno-exceptions',
            '-fsigned-char',

            '-Wall',
            '-Wextra',
            '-Wpointer-arith',
            '-Wno-unused-parameter',
            '-Wno-missing-field-initializers',
            '-Wno-reorder',
            '-Wno-redundant-decls',
            '-Wno-unknown-pragmas',
            '-Wno-expansion-to-defined',
            '-Werror=cast-align',
            '-Werror=attributes',
            '-Werror=format-security',
            '-Werror=format-extra-args',
            '-Werror=enum-compare',
            '-Werror=format',
            '-Werror=array-bounds',
            '-Werror=uninitialized',
            '-Werror=init-self',
            '-Werror=narrowing',
            '-Werror=return-type',
            '-Werror=switch',
            '-Werror=sign-compare',
            '-Werror=type-limits',
            '-Werror=undef',
            '-Werror=unused-result',
            '-Werror=shadow',
            '-Werror=unused-value',
            '-Werror=unused-variable',
            '-Werror=delete-non-virtual-dtor',
            '-Wfatal-errors',
            '-Wno-trigraphs',
            '-Werror=parentheses',
            '-DARDUPILOT_BUILD',
        ]

        if 'clang++' in cfg.env.COMPILER_CXX:
            env.CXXFLAGS += [
                '-fcolor-diagnostics',

                '-Werror=address-of-packed-member',

                '-Werror=inconsistent-missing-override',
                '-Werror=overloaded-virtual',

                # catch conversion issues:
                '-Werror=bitfield-enum-conversion',
                '-Werror=bool-conversion',
                '-Werror=constant-conversion',
                '-Werror=enum-conversion',
                '-Werror=int-conversion',
                '-Werror=literal-conversion',
                '-Werror=non-literal-null-conversion',
                '-Werror=null-conversion',
                '-Werror=objc-literal-conversion',
#                '-Werror=shorten-64-to-32',  # ARRAY_SIZE() creates this all over the place as the caller typically takes a uint32_t not a size_t
                '-Werror=string-conversion',
                #    '-Werror=sign-conversion', # can't use as we assign into AP_Int8 from uint8_ts

                '-Wno-gnu-designator',
                '-Wno-mismatched-tags',
                '-Wno-gnu-variable-sized-type-not-at-end',
                '-Werror=implicit-fallthrough',
            ]
        else:
            env.CXXFLAGS += [
                '-Wno-format-contains-nul',
                '-Werror=unused-but-set-variable'
            ]
            if self.cc_version_gte(cfg, 5, 2):
                env.CXXFLAGS += [
                    '-Werror=suggest-override',
                ]
            if self.cc_version_gte(cfg, 7, 4):
                env.CXXFLAGS += [
                    '-Werror=implicit-fallthrough',
                ]

        if cfg.options.Werror:
            errors = ['-Werror',
                      '-Werror=missing-declarations',
                      '-Werror=float-equal',
                      '-Werror=undef',
                    ]
            env.CFLAGS += errors
            env.CXXFLAGS += errors

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

        if self.with_can:
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

        if cfg.options.build_dates:
            env.build_dates = True

        # We always want to use PRI format macros
        cfg.define('__STDC_FORMAT_MACROS', 1)

        if cfg.options.disable_ekf2:
            env.CXXFLAGS += ['-DHAL_NAVEKF2_AVAILABLE=0']

        if cfg.options.disable_ekf3:
            env.CXXFLAGS += ['-DHAL_NAVEKF3_AVAILABLE=0']

        if cfg.options.osd or cfg.options.osd_fonts:
            env.CXXFLAGS += ['-DOSD_ENABLED=1', '-DHAL_MSP_ENABLED=1']

        if cfg.options.osd_fonts:
            for f in os.listdir('libraries/AP_OSD/fonts'):
                if fnmatch.fnmatch(f, "font*bin"):
                    env.ROMFS_FILES += [(f,'libraries/AP_OSD/fonts/'+f)]
            
    def pre_build(self, bld):
        '''pre-build hook that gets called before dynamic sources'''
        if bld.env.ROMFS_FILES:
            self.embed_ROMFS_files(bld)

    def build(self, bld):
        bld.ap_version_append_str('GIT_VERSION', bld.git_head_hash(short=True))
        import time
        ltime = time.localtime()
        if bld.env.build_dates:
            bld.ap_version_append_int('BUILD_DATE_YEAR', ltime.tm_year)
            bld.ap_version_append_int('BUILD_DATE_MONTH', ltime.tm_mon)
            bld.ap_version_append_int('BUILD_DATE_DAY', ltime.tm_mday)

    def embed_ROMFS_files(self, ctx):
        '''embed some files using AP_ROMFS'''
        import embed
        header = ctx.bldnode.make_node('ap_romfs_embedded.h').abspath()
        if not embed.create_embedded_h(header, ctx.env.ROMFS_FILES, ctx.env.ROMFS_UNCOMPRESSED):
            ctx.fatal("Failed to created ap_romfs_embedded.h")

Board = BoardMeta('Board', Board.__bases__, dict(Board.__dict__))

def add_dynamic_boards():
    '''add boards based on existance of hwdef.dat in subdirectories for ChibiOS'''
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ChibiOS/hwdef'))
    for d in dirlist:
        if d in _board_classes.keys():
            continue
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        if os.path.exists(hwdef):
            newclass = type(d, (chibios,), {'name': d})

def get_boards_names():
    add_dynamic_boards()

    return sorted(list(_board_classes.keys()), key=str.lower)

def get_removed_boards():
    '''list of boards which have been removed'''
    return sorted(['px4-v1', 'px4-v2', 'px4-v3', 'px4-v4', 'px4-v4pro'])

@conf
def get_board(ctx):
    global _board
    if not _board:
        if not ctx.env.BOARD:
            ctx.fatal('BOARD environment variable must be set before first call to get_board()')
        if ctx.env.BOARD in get_removed_boards():
            ctx.fatal('''
The board target %s has been removed from ArduPilot with the removal of NuttX support and HAL_PX4.

Please use a replacement build as follows:

 px4-v2     Use Pixhawk1 build
 px4-v3     Use Pixhawk1 or CubeBlack builds
 px4-v4     Use Pixracer build
 px4-v4pro  Use DrotekP3Pro build
''' % ctx.env.BOARD)

        boards = _board_classes.keys()
        if not ctx.env.BOARD in boards:
            ctx.fatal("Invalid board '%s': choices are %s" % (ctx.env.BOARD, ', '.join(sorted(boards, key=str.lower))))
        _board = _board_classes[ctx.env.BOARD]()
    return _board

# NOTE: Keeping all the board definitions together so we can easily
# identify opportunities to simplify common flags. In the future might
# be worthy to keep board definitions in files of their own.

class sitl(Board):

    def __init__(self):
        if Utils.unversioned_sys_platform().startswith("linux"):
            self.with_can = True
        else:
            self.with_can = False

    def configure_env(self, cfg, env):
        super(sitl, self).configure_env(cfg, env)
        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_SITL',
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_NONE',
            AP_SCRIPTING_CHECKS = 1, # SITL should always do runtime scripting checks
        )


        if self.with_can:
            cfg.define('HAL_NUM_CAN_IFACES', 2)
            cfg.define('UAVCAN_EXCEPTIONS', 0)

        env.CXXFLAGS += [
            '-Werror=float-equal'
        ]

        if not cfg.env.DEBUG:
            env.CXXFLAGS += [
                '-O3',
            ]

        if 'clang++' in cfg.env.COMPILER_CXX and cfg.options.asan:
            env.CXXFLAGS += [
                '-fsanitize=address',
                '-fno-omit-frame-pointer',
            ]

        env.LIB += [
            'm',
        ]

        cfg.check_librt(env)
        cfg.check_feenableexcept()

        env.LINKFLAGS += ['-pthread',]

        if cfg.env.DEBUG and 'clang++' in cfg.env.COMPILER_CXX and cfg.options.asan:
             env.LINKFLAGS += ['-fsanitize=address']

        env.AP_LIBRARIES += [
            'AP_HAL_SITL',
        ]

        if not cfg.env.AP_PERIPH:
            env.AP_LIBRARIES += [
                'SITL',
            ]

        if cfg.options.enable_sfml:
            if not cfg.check_SFML(env):
                cfg.fatal("Failed to find SFML libraries")

        if cfg.options.enable_sfml_joystick:
            if not cfg.check_SFML(env):
                cfg.fatal("Failed to find SFML libraries")
            env.CXXFLAGS += ['-DSFML_JOYSTICK']

        if cfg.options.sitl_osd:
            env.CXXFLAGS += ['-DWITH_SITL_OSD','-DOSD_ENABLED=1']
            for f in os.listdir('libraries/AP_OSD/fonts'):
                if fnmatch.fnmatch(f, "font*bin"):
                    env.ROMFS_FILES += [(f,'libraries/AP_OSD/fonts/'+f)]

        for f in os.listdir('Tools/autotest/models'):
            if fnmatch.fnmatch(f, "*.json") or fnmatch.fnmatch(f, "*.parm"):
                env.ROMFS_FILES += [('models/'+f,'Tools/autotest/models/'+f)]

        # include locations.txt so SITL on windows can lookup by name
        env.ROMFS_FILES += [('locations.txt','Tools/autotest/locations.txt')]

        # embed any scripts from ROMFS/scripts
        if os.path.exists('ROMFS/scripts'):
            for f in os.listdir('ROMFS/scripts'):
                if fnmatch.fnmatch(f, "*.lua"):
                    env.ROMFS_FILES += [('scripts/'+f,'ROMFS/scripts/'+f)]

        if len(env.ROMFS_FILES) > 0:
            env.CXXFLAGS += ['-DHAL_HAVE_AP_ROMFS_EMBEDDED_H']

        if cfg.options.sitl_rgbled:
            env.CXXFLAGS += ['-DWITH_SITL_RGBLED']

        if cfg.options.enable_sfml_audio:
            if not cfg.check_SFML_Audio(env):
                cfg.fatal("Failed to find SFML Audio libraries")
            env.CXXFLAGS += ['-DWITH_SITL_TONEALARM']

        if cfg.options.sitl_flash_storage:
            env.CXXFLAGS += ['-DSTORAGE_USE_FLASH=1']

        if cfg.env.DEST_OS == 'cygwin':
            env.LIB += [
                'winmm',
            ]

        if Utils.unversioned_sys_platform() == 'cygwin':
            env.CXXFLAGS += ['-DCYGWIN_BUILD']

        if 'clang++' in cfg.env.COMPILER_CXX:
            print("Disabling SLP for clang++")
            env.CXXFLAGS += [
                '-fno-slp-vectorize' # compiler bug when trying to use SLP
            ]
        
        def srcpath(path):
            return cfg.srcnode.make_node(path).abspath()
        env.SRCROOT = srcpath('')

class sitl_periph_gps(sitl):
    def configure_env(self, cfg, env):
        cfg.env.AP_PERIPH = 1
        cfg.env.DISABLE_SCRIPTING = 1
        super(sitl_periph_gps, self).configure_env(cfg, env)
        env.DEFINES.update(
            HAL_BUILD_AP_PERIPH = 1,
            PERIPH_FW = 1,
            CAN_APP_NODE_NAME = '"org.ardupilot.ap_periph_gps"',
            HAL_PERIPH_ENABLE_GPS = 1,
            HAL_WITH_DSP = 1,
            HAL_CAN_DEFAULT_NODE_ID = 0,
            HAL_RAM_RESERVE_START = 0,
            APJ_BOARD_ID = 100,
            HAL_NO_GCS = 1,
            HAL_NO_LOGGING = 1,
        )
        # libcanard is written for 32bit platforms
        env.CXXFLAGS += [
            '-m32',
        ]
        env.CFLAGS += [
            '-m32',
        ]
        env.LDFLAGS += [
            '-m32',
        ]


class chibios(Board):
    abstract = True
    toolchain = 'arm-none-eabi'

    def configure_env(self, cfg, env):
        super(chibios, self).configure_env(cfg, env)

        cfg.load('chibios')
        env.BOARD = self.name

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_CHIBIOS',
            HAVE_STD_NULLPTR_T = 0,
            USE_LIBC_REALLOC = 0,
        )

        env.AP_LIBRARIES += [
            'AP_HAL_ChibiOS',
        ]

        # make board name available for USB IDs
        env.CHIBIOS_BOARD_NAME = 'HAL_BOARD_NAME="%s"' % self.name
        env.CFLAGS += cfg.env.CPU_FLAGS + [
            '-Wno-cast-align',
            '-Wlogical-op',
            '-Wframe-larger-than=1300',
            '-fsingle-precision-constant',
            '-Wno-attributes',
            '-fno-exceptions',
            '-Wall',
            '-Wextra',
            '-Wno-sign-compare',
            '-Wfloat-equal',
            '-Wpointer-arith',
            '-Wmissing-declarations',
            '-Wno-unused-parameter',
            '-Werror=array-bounds',
            '-Wfatal-errors',
            '-Werror=uninitialized',
            '-Werror=init-self',
            '-Werror=unused-but-set-variable',
            '-Wno-missing-field-initializers',
            '-Wno-trigraphs',
            '-fno-strict-aliasing',
            '-fomit-frame-pointer',
            '-falign-functions=16',
            '-ffunction-sections',
            '-fdata-sections',
            '-fno-strength-reduce',
            '-fno-builtin-printf',
            '-fno-builtin-fprintf',
            '-fno-builtin-vprintf',
            '-fno-builtin-vfprintf',
            '-fno-builtin-puts',
            '-mno-thumb-interwork',
            '-mthumb',
            '--specs=nano.specs',
            '-specs=nosys.specs',
            '-DCHIBIOS_BOARD_NAME="%s"' % self.name,
            '-D__USE_CMSIS',
            '-Werror=deprecated-declarations'
        ]
        if not cfg.options.Werror:
            env.CFLAGS += [
            '-Wno-error=double-promotion',
            '-Wno-error=missing-declarations',
            '-Wno-error=float-equal',
            '-Wno-error=undef',
            '-Wno-error=cpp',
            ]

        env.CXXFLAGS += env.CFLAGS + [
            '-fno-rtti',
            '-fno-threadsafe-statics',
        ]
        env.CFLAGS += [
            '-std=c11'
        ]

        if Utils.unversioned_sys_platform() == 'cygwin':
            env.CXXFLAGS += ['-DCYGWIN_BUILD']

        bldnode = cfg.bldnode.make_node(self.name)
        env.BUILDROOT = bldnode.make_node('').abspath()
        env.LINKFLAGS = cfg.env.CPU_FLAGS + [
            '-fomit-frame-pointer',
            '-falign-functions=16',
            '-ffunction-sections',
            '-fdata-sections',
            '-u_port_lock',
            '-u_port_unlock',
            '-u_exit',
            '-u_kill',
            '-u_getpid',
            '-u_errno',
            '-uchThdExit',
            '-fno-common',
            '-nostartfiles',
            '-mno-thumb-interwork',
            '-mthumb',
            '-specs=nano.specs',
            '-specs=nosys.specs',
            '-L%s' % env.BUILDROOT,
            '-L%s' % cfg.srcnode.make_node('modules/ChibiOS/os/common/startup/ARMCMx/compilers/GCC/ld/').abspath(),
            '-L%s' % cfg.srcnode.make_node('libraries/AP_HAL_ChibiOS/hwdef/common/').abspath(),
            '-Wl,--gc-sections,--no-warn-mismatch,--library-path=/ld,--script=ldscript.ld,--defsym=__process_stack_size__=%s,--defsym=__main_stack_size__=%s' % (cfg.env.PROCESS_STACK, cfg.env.MAIN_STACK)
        ]

        if cfg.env.DEBUG:
            env.CFLAGS += [
                '-gdwarf-4',
                '-g3',
            ]
            env.LINKFLAGS += [
                '-gdwarf-4',
                '-g3',
            ]

        if cfg.env.ENABLE_ASSERTS:
            cfg.msg("Enabling ChibiOS asserts", "yes")
            env.CFLAGS += [ '-DHAL_CHIBIOS_ENABLE_ASSERTS' ]
            env.CXXFLAGS += [ '-DHAL_CHIBIOS_ENABLE_ASSERTS' ]
        else:
            cfg.msg("Enabling ChibiOS asserts", "no")

        if cfg.env.ENABLE_MALLOC_GUARD:
            cfg.msg("Enabling malloc guard", "yes")
            env.CFLAGS += [ '-DHAL_CHIBIOS_ENABLE_MALLOC_GUARD' ]
            env.CXXFLAGS += [ '-DHAL_CHIBIOS_ENABLE_MALLOC_GUARD' ]
        else:
            cfg.msg("Enabling malloc guard", "no")
            
        env.LIB += ['gcc', 'm']

        env.GIT_SUBMODULES += [
            'ChibiOS',
        ]

        env.INCLUDES += [
            cfg.srcnode.find_dir('libraries/AP_GyroFFT/CMSIS_5/include').abspath()
        ]

        # whitelist of compilers which we should build with -Werror
        gcc_whitelist = [
            ('4','9','3'),
            ('6','3','1'),
            ('9','2','1'),
            ('9','3','1'),
        ]

        if cfg.options.Werror or cfg.env.CC_VERSION in gcc_whitelist:
            cfg.msg("Enabling -Werror", "yes")
            if '-Werror' not in env.CXXFLAGS:
                env.CXXFLAGS += [ '-Werror' ]
        else:
            cfg.msg("Enabling -Werror", "no")

        try:
            import intelhex
            env.HAVE_INTEL_HEX = True
            cfg.msg("Checking for intelhex module:", 'OK')
        except Exception:
            cfg.msg("Checking for intelhex module:", 'disabled', color='YELLOW')
            env.HAVE_INTEL_HEX = False

    def build(self, bld):
        super(chibios, self).build(bld)
        bld.ap_version_append_str('CHIBIOS_GIT_VERSION', bld.git_submodule_head_hash('ChibiOS', short=True))
        bld.load('chibios')

    def pre_build(self, bld):
        '''pre-build hook that gets called before dynamic sources'''
        from waflib.Context import load_tool
        module = load_tool('chibios', [], with_sys_path=True)
        fun = getattr(module, 'pre_build', None)
        if fun:
            fun(bld)
        super(chibios, self).pre_build(bld)

class linux(Board):
    def configure_env(self, cfg, env):
        super(linux, self).configure_env(cfg, env)

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
        env.AP_LIBRARIES += [
            'AP_HAL_Linux',
        ]

        if self.with_can:
            cfg.define('UAVCAN_EXCEPTIONS', 0)

        if cfg.options.apstatedir:
            cfg.define('AP_STATEDIR', cfg.options.apstatedir)

    def build(self, bld):
        super(linux, self).build(bld)
        if bld.options.upload:
            waflib.Options.commands.append('rsync')
            # Avoid infinite recursion
            bld.options.upload = False

class navigator(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(navigator, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE='HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR',
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

class edge(linux):
    toolchain = 'arm-linux-gnueabihf'

    def __init__(self):
        self.with_can = True

    def configure_env(self, cfg, env):
        super(edge, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_EDGE',
        )

class zynq(linux):
    toolchain = 'arm-xilinx-linux-gnueabi'

    def configure_env(self, cfg, env):
        super(zynq, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_ZYNQ',
        )

class ocpoc_zynq(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(ocpoc_zynq, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_OCPOC_ZYNQ',
        )

class bbbmini(linux):
    toolchain = 'arm-linux-gnueabihf'

    def __init__(self):
        self.with_can = True

    def configure_env(self, cfg, env):
        super(bbbmini, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BBBMINI',
        )

class blue(linux):
    toolchain = 'arm-linux-gnueabihf'

    def __init__(self):
        self.with_can = True

    def configure_env(self, cfg, env):
        super(blue, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_BLUE',
        )

class pocket(linux):
    toolchain = 'arm-linux-gnueabihf'

    def __init__(self):
        self.with_can = True

    def configure_env(self, cfg, env):
        super(pocket, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_POCKET',
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

class pxfmini(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(pxfmini, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_PXFMINI',
        )

class aero(linux):
    def __init__(self):
        self.with_can = True

    def configure_env(self, cfg, env):
        super(aero, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_AERO',
        )

class rst_zynq(linux):
    toolchain = 'arm-linux-gnueabihf'

    def configure_env(self, cfg, env):
        super(rst_zynq, self).configure_env(cfg, env)

        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_LINUX_RST_ZYNQ',
        )

class SITL_static(sitl):
    def configure_env(self, cfg, env):
        super(SITL_static, self).configure_env(cfg, env)
        cfg.env.STATIC_LINKING = True

class SITL_x86_64_linux_gnu(SITL_static):
    toolchain = 'x86_64-linux-gnu'

class SITL_arm_linux_gnueabihf(SITL_static):
    toolchain = 'arm-linux-gnueabihf'

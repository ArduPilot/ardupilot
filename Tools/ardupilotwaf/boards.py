#!/usr/bin/env python
# encoding: utf-8

from collections import OrderedDict
import sys, os

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
        self.with_uavcan = False

    def configure(self, cfg):
        cfg.env.TOOLCHAIN = self.toolchain
        cfg.env.ROMFS_FILES = []
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
            '-Wno-trigraphs',
            '-Werror=return-type',
            '-Werror=unused-result',
            '-Werror=narrowing',
        ]

        if cfg.options.enable_scripting:
            env.DEFINES.update(
                ENABLE_SCRIPTING = 1,
                LUA_32BITS = 1,
                )

            env.ROMFS_FILES += [
                ('sandbox.lua', 'libraries/AP_Scripting/scripts/sandbox.lua'),
                ]

            env.AP_LIBRARIES += [
                'AP_Scripting',
                'AP_Scripting/lua/src',
                ]

            env.CXXFLAGS += [
                '-DHAL_HAVE_AP_ROMFS_EMBEDDED_H'
                ]

        if cfg.options.scripting_checks:
            env.DEFINES.update(
                AP_SCRIPTING_CHECKS = 1,
                )

        if 'clang' in cfg.env.COMPILER_CC:
            env.CFLAGS += [
                '-fcolor-diagnostics',

                '-Wno-gnu-designator',
                '-Wno-inconsistent-missing-override',
                '-Wno-mismatched-tags',
                '-Wno-gnu-variable-sized-type-not-at-end',
            ]

        if cfg.env.DEBUG:
            env.CFLAGS += [
                '-g',
                '-O0',
            ]

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
            '-Werror=narrowing',
            '-Werror=return-type',
            '-Werror=switch',
            '-Werror=sign-compare',
            '-Werror=unused-result',
            '-Werror=return-type',
            '-Wfatal-errors',
            '-Wno-trigraphs',
        ]

        if 'clang++' in cfg.env.COMPILER_CXX:
            env.CXXFLAGS += [
                '-fcolor-diagnostics',

                '-Wno-gnu-designator',
                '-Wno-inconsistent-missing-override',
                '-Wno-mismatched-tags',
                '-Wno-gnu-variable-sized-type-not-at-end',
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


    def pre_build(self, bld):
        '''pre-build hook that gets called before dynamic sources'''
        if bld.env.ROMFS_FILES:
            self.embed_ROMFS_files(bld)

    def build(self, bld):
        bld.ap_version_append_str('GIT_VERSION', bld.git_head_hash(short=True))
        import time
        ltime = time.localtime()
        bld.ap_version_append_int('BUILD_DATE_YEAR', ltime.tm_year)
        bld.ap_version_append_int('BUILD_DATE_MONTH', ltime.tm_mon)
        bld.ap_version_append_int('BUILD_DATE_DAY', ltime.tm_mday)

    def embed_ROMFS_files(self, ctx):
        '''embed some files using AP_ROMFS'''
        import embed
        if ctx.env.USE_NUTTX_IOFW:
            # use fmuv2_IO_NuttX.bin instead of fmuv2_IO.bin
            for i in range(len(ctx.env.ROMFS_FILES)):
                (name,filename) = ctx.env.ROMFS_FILES[i]
                if name == 'io_firmware.bin':
                    filename = 'Tools/IO_Firmware/fmuv2_IO_NuttX.bin'
                    print("Using IO firmware %s" % filename)
                    ctx.env.ROMFS_FILES[i] = (name,filename);
        header = ctx.bldnode.make_node('ap_romfs_embedded.h').abspath()
        if not embed.create_embedded_h(header, ctx.env.ROMFS_FILES):
            bld.fatal("Failed to created ap_romfs_embedded.h")

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
            AP_SCRIPTING_CHECKS = 1, # SITL should always do runtime scripting checks
        )

        env.CXXFLAGS += [
            '-Werror=float-equal'
        ]

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

        if cfg.options.enable_sfml:
            if not cfg.check_SFML(env):
                cfg.fatal("Failed to find SFML libraries")
            env.CXXFLAGS += ['-DWITH_SITL_OSD','-DOSD_ENABLED=ENABLED','-DHAL_HAVE_AP_ROMFS_EMBEDDED_H']
            import fnmatch
            for f in os.listdir('libraries/AP_OSD/fonts'):
                if fnmatch.fnmatch(f, "font*bin"):
                    env.ROMFS_FILES += [(f,'libraries/AP_OSD/fonts/'+f)]

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

class chibios(Board):
    abstract = True
    toolchain = 'arm-none-eabi'

    def configure_env(self, cfg, env):
        super(chibios, self).configure_env(cfg, env)

        cfg.load('chibios')
        env.BOARD = self.name

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_CHIBIOS',
            HAVE_OCLOEXEC = 0,
            HAVE_STD_NULLPTR_T = 0,
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
            '-Wno-error=double-promotion',
            '-Wno-error=missing-declarations',
            '-Wno-error=float-equal',
            '-Wno-error=undef',
            '-Wno-error=cpp',
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
            '-Werror=unused-variable',
            '-Werror=uninitialized',
            '-Werror=init-self',
            '-Wframe-larger-than=1024',
            '-Werror=unused-but-set-variable',
            '-Wno-missing-field-initializers',
            '-Wno-trigraphs',
            '-Os',
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
        ]
        env.CXXFLAGS += env.CFLAGS + [
            '-fno-rtti',
            '-fno-threadsafe-statics',
        ]

        if Utils.unversioned_sys_platform() == 'cygwin':
            env.CXXFLAGS += ['-DCYGWIN_BUILD']

        bldnode = cfg.bldnode.make_node(self.name)
        env.BUILDROOT = bldnode.make_node('').abspath()
        env.LINKFLAGS = cfg.env.CPU_FLAGS + [
            '-Os',
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

        env.LIB += ['gcc', 'm']

        env.GIT_SUBMODULES += [
            'ChibiOS',
        ]

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
        super(chibios, self).pre_build(bld)
        from waflib.Context import load_tool
        module = load_tool('chibios', [], with_sys_path=True)
        fun = getattr(module, 'pre_build', None)
        if fun:
            fun(bld)

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

        if self.with_uavcan:
            cfg.define('UAVCAN_EXCEPTIONS', 0)

        if cfg.options.apstatedir:
            cfg.define('AP_STATEDIR', cfg.options.apstatedir)

    def build(self, bld):
        super(linux, self).build(bld)
        if bld.options.upload:
            waflib.Options.commands.append('rsync')
            # Avoid infinite recursion
            bld.options.upload = False

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
        self.with_uavcan = True

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

class pocket(linux):
    toolchain = 'arm-linux-gnueabihf'

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
        self.with_uavcan = True

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

        # Path relative to the ROMFS directory where to find a file with default
        # parameters. If set this file will be copied to /etc/defaults.parm
        # inside the ROMFS
        self.param_defaults = None

        self.ROMFS_EXCLUDE = []

        # use ardupilot version of uploader.py
        os.environ['UPLOADER'] = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', 'scripts', 'uploader.py'))

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
            '-Wno-attributes',
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

        if sys.platform == 'cygwin':
            env.CXXFLAGS += ['-DCYGWIN_BUILD']

        env.ROMFS_EXCLUDE = self.ROMFS_EXCLUDE

        env.PX4_BOOTLOADER_NAME = self.bootloader_name
        env.PX4_BOARD_NAME = self.board_name
        env.PX4_BOARD_RC = self.board_rc
        env.PX4_PX4IO_NAME = self.px4io_name
        env.PX4_PARAM_DEFAULTS = self.param_defaults
        env.PX4_RC_S_SCRIPT = 'init.d/rcS'

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

class skyviper_v2450_px4(px4_v3):
    name = 'skyviper-v2450-px4'
    def __init__(self):
        super(skyviper_v2450_px4, self).__init__()
        self.px4io_name = None
        self.param_defaults = '../../../Tools/Frame_params/SkyViper-2450GPS/defaults.parm'

    def configure_env(self, cfg, env):
        super(skyviper_v2450_px4, self).configure_env(cfg, env)

        env.DEFINES.update(
            TOY_MODE_ENABLED = 'ENABLED',
            USE_FLASH_STORAGE = 1,
            ARMING_DELAY_SEC = 0,
            LAND_START_ALT = 700,
            HAL_RCINPUT_WITH_AP_RADIO = 1,
            LAND_DETECTOR_ACCEL_MAX = 2,
            CYRF_SPI_PX4_SPI_BUS = 2,
            CYRF_SPI_PX4_SPIDEV_EXT = '(spi_dev_e)1',
            CYRF_IRQ_INPUT = '(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)',
        )
        env.PX4_RC_S_SCRIPT = 'init.d/rcS_no_microSD'
        env.BUILD_ABIN = True

class px4_v4(px4):
    name = 'px4-v4'
    def __init__(self):
        super(px4_v4, self).__init__()
        self.bootloader_name = 'px4fmuv4_bl.bin'
        self.board_name = 'px4fmu-v4'
        self.romfs_exclude(['oreoled.bin'])
        self.with_uavcan = True

class px4_v4pro(px4):
    name = 'px4-v4pro'
    def __init__(self):
        super(px4_v4pro, self).__init__()
        self.bootloader_name = 'px4fmuv4pro_bl.bin'
        self.board_name = 'px4fmu-v4pro'
        self.px4io_name = 'px4io-v2'
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
        self.param_defaults = '../../../Tools/Frame_params/intel-aero-rtf.param'

# encoding: utf-8

# flake8: noqa

from collections import OrderedDict
import re
import sys, os
import fnmatch
import platform
import glob

import waflib
from waflib import Utils, Context
from waflib.Configure import conf
import json
_board_classes = {}
_board = None

# modify our search path:
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_ChibiOS/hwdef/scripts'))
import chibios_hwdef
import build_options

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
        self.with_littlefs = False

    def configure(self, cfg):
        cfg.env.TOOLCHAIN = cfg.options.toolchain or self.toolchain
        if hasattr(self,'configure_toolchain'):
            self.configure_toolchain(cfg)
        else:
            cfg.load('toolchain')
        cfg.load('cxx_checks')

        # don't check elf symbols by default
        cfg.env.CHECK_SYMBOLS = False

        env = waflib.ConfigSet.ConfigSet()
        def srcpath(path):
            return cfg.srcnode.make_node(path).abspath()
        env.SRCROOT = srcpath('')

        self.configure_env(cfg, env)

        # Setup scripting:
        env.DEFINES.update(
            LUA_32BITS = 1,
        )

        env.AP_LIBRARIES += [
            'AP_Scripting',
            'AP_Scripting/lua/src',
        ]

        if cfg.options.enable_scripting:
            env.DEFINES.update(
                AP_SCRIPTING_ENABLED = 1,
            )
        elif cfg.options.disable_scripting:
            env.DEFINES.update(
                AP_SCRIPTING_ENABLED = 0,
            )

        # embed any scripts from ROMFS/scripts
        if os.path.exists('ROMFS/scripts'):
            for f in os.listdir('ROMFS/scripts'):
                if fnmatch.fnmatch(f, "*.lua"):
                    env.ROMFS_FILES += [('scripts/'+f,'ROMFS/scripts/'+f)]

        # allow GCS disable for AP_DAL example
        if cfg.options.no_gcs:
            env.CXXFLAGS += ['-DHAL_GCS_ENABLED=0']

        # Setup DDS
        if env.BOARD_CLASS == "ChibiOS" or env.BOARD_CLASS == "Linux":
            # need to check the hwdef.h file for the board to see if dds is enabled
            # the issue here is that we need to configure the env properly to include
            # the DDS library, but the definition is the the hwdef file
            # and can be overridden by the commandline options
            with open(env.BUILDROOT + "/hwdef.h", 'r', encoding="utf8") as file:
                if "#define AP_DDS_ENABLED 1" in file.read():
                    # Enable DDS if the hwdef file has it enabled
                    cfg.env.OPTIONS['enable_DDS'] = True
                elif cfg.env.OPTIONS.get('enable_DDS', False):
                    # Add the define enabled if the hwdef file does not have it and the commandline option is set
                    env.DEFINES.update(
                        AP_DDS_ENABLED=1,
                    )
                else:
                    # Add the define disabled if the hwdef file does not have it and the commandline option is not set
                    env.DEFINES.update(
                        AP_DDS_ENABLED=0,
                    )
        else:
            if cfg.options.enable_DDS:
                env.DEFINES.update(
                    AP_DDS_ENABLED=1,
                )
            else:
                env.DEFINES.update(
                    AP_DDS_ENABLED=0,
                )

        # setup for supporting onvif cam control
        if cfg.options.enable_onvif:
            cfg.recurse('libraries/AP_ONVIF')
            env.ENABLE_ONVIF = True
            env.ROMFS_FILES += [('scripts/ONVIF_Camera_Control.lua',
                                'libraries/AP_Scripting/applets/ONVIF_Camera_Control.lua')]
            env.DEFINES.update(
                ENABLE_ONVIF=1,
                SCRIPTING_ENABLE_DEFAULT=1,
            )
            env.AP_LIBRARIES += [
                'AP_ONVIF'
            ]
        else:
            env.ENABLE_ONVIF = False
            env.DEFINES.update(
                ENABLE_ONVIF=0,
            )

        # allow enable of OpenDroneID for any board
        if cfg.options.enable_opendroneid:
            env.ENABLE_OPENDRONEID = True
            env.DEFINES.update(
                AP_OPENDRONEID_ENABLED=1,
            )
            cfg.msg("Enabled OpenDroneID", 'yes')
        else:
            cfg.msg("Enabled OpenDroneID", 'no', color='YELLOW')

        # allow enable of firmware ID checking for any board
        if cfg.options.enable_check_firmware:
            env.CHECK_FIRMWARE_ENABLED = True
            env.DEFINES.update(
                AP_CHECK_FIRMWARE_ENABLED=1,
            )
            cfg.msg("Enabled firmware ID checking", 'yes')
        else:
            cfg.msg("Enabled firmware ID checking", 'no', color='YELLOW')

        if cfg.options.enable_gps_logging:
            env.DEFINES.update(
                AP_GPS_DEBUG_LOGGING_ENABLED=1,
            )
            cfg.msg("GPS Debug Logging", 'yes')
        else:
            cfg.msg("GPS Debug Logging", 'no', color='YELLOW')

        # allow enable of custom controller for any board
        # enabled on sitl by default
        if (cfg.options.enable_custom_controller or self.get_name() == "sitl") and not cfg.options.no_gcs:
            env.ENABLE_CUSTOM_CONTROLLER = True
            env.DEFINES.update(
                AP_CUSTOMCONTROL_ENABLED=1,
            )
            env.AP_LIBRARIES += [
                'AC_CustomControl'
            ]
            cfg.msg("Enabled custom controller", 'yes')
        else:
            env.DEFINES.update(
                AP_CUSTOMCONTROL_ENABLED=0,
            )
            cfg.msg("Enabled custom controller", 'no', color='YELLOW')

        # support enabling any option in build_options.py
        for opt in build_options.BUILD_OPTIONS:
            enable_option = opt.config_option().replace("-","_")
            disable_option = "disable_" + enable_option[len("enable-"):]
            lower_disable_option = disable_option.lower().replace("_", "-")
            lower_enable_option = enable_option.lower().replace("_", "-")
            if getattr(cfg.options, enable_option, False) or getattr(cfg.options, lower_enable_option, False):
                env.CXXFLAGS += ['-D%s=1' % opt.define]
                cfg.msg("Enabled %s" % opt.label, 'yes', color='GREEN')
            elif getattr(cfg.options, disable_option, False) or getattr(cfg.options, lower_disable_option, False):
                env.CXXFLAGS += ['-D%s=0' % opt.define]
                cfg.msg("Enabled %s" % opt.label, 'no', color='YELLOW')

        # support embedding lua drivers and applets
        driver_list = glob.glob(os.path.join(Context.run_dir, "libraries/AP_Scripting/drivers/*.lua"))
        applet_list = glob.glob(os.path.join(Context.run_dir, "libraries/AP_Scripting/applets/*.lua"))
        for d in driver_list + applet_list:
            bname = os.path.basename(d)
            embed_name = bname[:-4]
            embed_option = f"embed-{embed_name}".replace("-","_")
            if getattr(cfg.options, embed_option, False):
                env.ROMFS_FILES += [(f'scripts/{bname}', d)]
                cfg.msg(f"Embedded {bname}", 'yes', color='GREEN')

        if cfg.options.disable_networking:
            env.CXXFLAGS += ['-DAP_NETWORKING_ENABLED=0']

        if cfg.options.enable_networking_tests:
            env.CXXFLAGS += ['-DAP_NETWORKING_TESTS_ENABLED=1']

        if cfg.options.enable_iomcu_profiled_support:
            env.CXXFLAGS += ['-DAP_IOMCU_PROFILED_SUPPORT_ENABLED=1']

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
        if os.path.exists(os.path.join(env.SRCROOT, '.vscode/c_cpp_properties.json')) and 'AP_NO_COMPILE_COMMANDS' not in os.environ:
            # change c_cpp_properties.json configure the VSCode Intellisense env
            c_cpp_properties = json.load(open(os.path.join(env.SRCROOT, '.vscode/c_cpp_properties.json')))
            for config in c_cpp_properties['configurations']:
                config['compileCommands'] = "${workspaceFolder}/build/%s/compile_commands.json" % self.get_name()
            json.dump(c_cpp_properties, open(os.path.join(env.SRCROOT, './.vscode/c_cpp_properties.json'), 'w'), indent=4)
            cfg.msg("Configured VSCode Intellisense", 'yes')
        else:
            cfg.msg("Configured VSCode Intellisense:", 'no', color='YELLOW')

    def cc_version_gte(self, cfg, want_major, want_minor):
        if cfg.env.TOOLCHAIN == "custom":
            return True
        (major, minor, patchlevel) = cfg.env.CC_VERSION
        return (int(major) > want_major or
                (int(major) == want_major and int(minor) >= want_minor))

    def configure_env(self, cfg, env):
        # Use a dictionary instead of the conventional list for definitions to
        # make easy to override them. Convert back to list before consumption.
        env.DEFINES = {}

        env.with_can = self.with_can

        # potentially set extra defines from an environment variable:
        if cfg.options.define is not None:
            for (n, v) in [d.split("=") for d in cfg.options.define]:
                cfg.msg("Defining: %s" % (n, ), v)
                env.CFLAGS += ['-D%s=%s' % (n, v)]
                env.CXXFLAGS += ['-D%s=%s' % (n, v)]

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

        if cfg.options.assert_cc_version:
            cfg.msg("Checking compiler", "%s %s"  % (cfg.options.assert_cc_version, ".".join(cfg.env.CC_VERSION)))
            have_version = cfg.env.COMPILER_CXX+"-"+'.'.join(list(cfg.env.CC_VERSION))
            want_version = cfg.options.assert_cc_version
            if have_version != want_version:
                cfg.fatal("cc version mismatch: %s should be %s" % (have_version, want_version))

        # ensure that if you are using clang you're using it for both
        # C and C++!
        if (("clang" in cfg.env.COMPILER_CC and "clang" not in cfg.env.COMPILER_CXX) or
            ("clang" not in cfg.env.COMPILER_CC and "clang" in cfg.env.COMPILER_CXX)):
            cfg.fatal("Compiler mismatch; set CC and CXX to matching compilers (eg. CXX=clang++-19 CC=clang-19")

        if 'clang' in cfg.env.COMPILER_CC:
            env.CFLAGS += [
                '-fcolor-diagnostics',
                '-Wno-gnu-designator',
                '-Wno-inconsistent-missing-override',
                '-Wno-mismatched-tags',
                '-Wno-gnu-variable-sized-type-not-at-end',
                '-Werror=implicit-fallthrough',
                '-cl-single-precision-constant',
                '-Wno-vla-extension',
                '-ftrapping-math',  # prevent re-ordering of sanity checks
            ]
        else:
            env.CFLAGS += [
                '-Wno-format-contains-nul',
                '-fsingle-precision-constant', # force const vals to be float , not double. so 100.0 means 100.0f
            ]

        if cfg.env.DEBUG:
            env.CFLAGS += [
                '-g',
                '-O0',
            ]
            env.DEFINES.update(
                HAL_DEBUG_BUILD = 1,
            )
        elif cfg.options.debug_symbols:
            env.CFLAGS += [
                '-g',
            ]
        if cfg.env.COVERAGE:
            env.CFLAGS += [
                '-fprofile-arcs',
                '-ftest-coverage',
            ]
            env.CXXFLAGS += [
                '-fprofile-arcs',
                '-ftest-coverage',
            ]
            env.LINKFLAGS += [
                '-lgcov',
                '-coverage',
            ]
            env.DEFINES.update(
                HAL_COVERAGE_BUILD = 1,
            )

        if cfg.options.bootloader:
            # don't let bootloaders try and pull scripting in
            cfg.options.disable_scripting = True

        if cfg.options.enable_math_check_indexes:
            env.CXXFLAGS += ['-DMATH_CHECK_INDEXES']

        if cfg.options.private_key:
            env.PRIVATE_KEY = cfg.options.private_key
            
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
            '-Wno-redundant-decls',
            '-Wno-unknown-pragmas',
            '-Wno-expansion-to-defined',
            '-Werror=reorder',
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
            '-Wuninitialized',
            '-Warray-bounds',
        ]

        use_prefix_map = False
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
                '-Wno-vla-extension',
                '-Wno-gnu-variable-sized-type-not-at-end',
                '-Werror=implicit-fallthrough',
                '-cl-single-precision-constant',
                '-ftrapping-math',  # prevent re-ordering of sanity checks
            ]
            if self.cc_version_gte(cfg, 10, 0):
                use_prefix_map = True
        else:
            env.CXXFLAGS += [
                '-Wno-format-contains-nul',
                '-Werror=unused-but-set-variable',
                '-fsingle-precision-constant',
                '-Wno-psabi',
            ]
            if self.cc_version_gte(cfg, 5, 2):
                env.CXXFLAGS += [
                    '-Werror=suggest-override',
                ]
            if self.cc_version_gte(cfg, 7, 4):
                env.CXXFLAGS += [
                    '-Werror=implicit-fallthrough',
                    '-Werror=maybe-uninitialized',
                    '-Werror=duplicated-cond',
                ]
            if self.cc_version_gte(cfg, 8, 0):
                use_prefix_map = True
            if self.cc_version_gte(cfg, 8, 4):
                env.CXXFLAGS += [
                    '-Werror=sizeof-pointer-div',
                ]
            if self.cc_version_gte(cfg, 13, 2):
                env.CXXFLAGS += [
                    '-Werror=use-after-free',
                ]
                env.CFLAGS += [
                    '-Werror=use-after-free',
                ]

        if cfg.env.TOOLCHAIN == "custom":
            # the QURT board stuff should be extracting the compiler
            # version properly.  In the meantime, here's a really
            # awesome thing:
            use_prefix_map = False

        if use_prefix_map:
            # fixes to make __FILE__ and debug paths repeatable in .elf/.bin

            # build root including variant (e.g. by default build/sitl)
            bldnode = cfg.bldnode.make_node(cfg.variant)
            # compute source root path from the perspective of the exact build
            # root above. this path is (as far as we can tell) what waf
            # prefixes source files with when passing them to the compiler
            file_prefix = str(cfg.srcnode.path_from(bldnode))

            # now tell the compiler to remap that prefix to `../..` (the prefix
            # by default) so any stored source paths are independent of
            # wherever the build dir is and the debugger can find the source
            cfg.env.CFLAGS += [f"-ffile-prefix-map={file_prefix}=../.."]
            cfg.env.CXXFLAGS += [f"-ffile-prefix-map={file_prefix}=../.."]

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
            if self.cc_version_gte(cfg, 15, 0):
                env.LINKFLAGS += [
                    '-Wl,-dead_strip,-ld_classic',
                ]
            else:
                env.LINKFLAGS += [
                    '-Wl,-dead_strip',
                ]
        else:
            env.LINKFLAGS += [
                '-fno-exceptions',
                '-Wl,--gc-sections',
            ]

        if self.with_can:
            # for both AP_Perip and main fw enable deadlines
            env.DEFINES.update(CANARD_ENABLE_DEADLINE = 1)

            if not cfg.env.AP_PERIPH:
                env.AP_LIBRARIES += [
                    'AP_DroneCAN',
                    'modules/DroneCAN/libcanard/*.c',
                    ]
                if cfg.options.enable_dronecan_tests:
                    env.DEFINES.update(AP_TEST_DRONECAN_DRIVERS = 1)

                env.DEFINES.update(
                    DRONECAN_CXX_WRAPPERS = 1,
                    USE_USER_HELPERS = 1,
                    CANARD_ALLOCATE_SEM=1
                )



        if cfg.options.build_dates:
            env.build_dates = True

        # We always want to use PRI format macros
        cfg.define('__STDC_FORMAT_MACROS', 1)

        if cfg.options.postype_single:
            env.CXXFLAGS += ['-DHAL_WITH_POSTYPE_DOUBLE=0']
            
        if cfg.options.osd or cfg.options.osd_fonts:
            env.CXXFLAGS += ['-DOSD_ENABLED=1', '-DHAL_MSP_ENABLED=1']

        if cfg.options.osd_fonts:
            for f in os.listdir('libraries/AP_OSD/fonts'):
                if fnmatch.fnmatch(f, "font*bin"):
                    env.ROMFS_FILES += [(f,'libraries/AP_OSD/fonts/'+f)]

        if cfg.options.ekf_double:
            env.CXXFLAGS += ['-DHAL_WITH_EKF_DOUBLE=1']

        if cfg.options.ekf_single:
            env.CXXFLAGS += ['-DHAL_WITH_EKF_DOUBLE=0']

        if cfg.env.CONSISTENT_BUILDS:
            # if symbols are renamed we don't want them to affect the output:
            env.CXXFLAGS += ['-fno-rtti']
            # avoid different filenames for the same source file
            # affecting the compiler output:
            env.CXXFLAGS += ['-frandom-seed=4']  # ob. xkcd

            # disable setting build ID in the ELF header. though if two binaries
            # are identical they will have the same build ID, setting it avoids
            # creating a difference if they are not in a way we care about.
            env.LDFLAGS += ['-Wl,--build-id=none']
            # squash all line numbers to be the number 17
            env.CXXFLAGS += [
                "-D__AP_LINE__=17",
            ]
        else:
            env.CXXFLAGS += [
                "-D__AP_LINE__=__LINE__",
            ]

        # add files from ROMFS_custom
        custom_dir = 'ROMFS_custom'
        if os.path.exists(custom_dir):
            for root, subdirs, files in os.walk(custom_dir):
                for f in files:
                    if fnmatch.fnmatch(f,"*~"):
                        # exclude emacs tmp files
                        continue
                    fname = root[len(custom_dir)+1:]+"/"+f
                    if fname.startswith("/"):
                        fname = fname[1:]
                    env.ROMFS_FILES += [(fname,root+"/"+f)]

    def pre_build(self, bld):
        '''pre-build hook that gets called before dynamic sources'''
        if bld.env.ROMFS_FILES:
            self.embed_ROMFS_files(bld)

    def build(self, bld):
        git_hash_ext = bld.git_head_hash(short=True, hash_abbrev=16)
        bld.ap_version_append_str('GIT_VERSION', bld.git_head_hash(short=True), "abcdef")
        bld.ap_version_append_str('GIT_VERSION_EXTENDED', git_hash_ext, "0123456789abcdef")
        bld.ap_version_append_int('GIT_VERSION_INT', int("0x" + bld.git_head_hash(short=True), base=16), 15)
        # this build root is mostly used as a temporary directory by SIM_JSBSim and probably needs to go
        bld.ap_version_append_str('AP_BUILD_ROOT', bld.srcnode.abspath(), "/tmp")

        if bld.env.build_dates:
            if bld.options.consistent_builds:
                raise ValueError("can't enable consistent builds and build dates")

            import time
            ltime = time.localtime()
            bld.ap_version_append_int('BUILD_DATE_YEAR', ltime.tm_year)
            bld.ap_version_append_int('BUILD_DATE_MONTH', ltime.tm_mon)
            bld.ap_version_append_int('BUILD_DATE_DAY', ltime.tm_mday)

    def embed_ROMFS_files(self, ctx):
        '''embed some files using AP_ROMFS'''
        import embed
        header = ctx.bldnode.make_node('ap_romfs_embedded.h').abspath()
        if not embed.create_embedded_h(header, ctx.env.ROMFS_FILES, ctx.env.ROMFS_UNCOMPRESSED):
            ctx.fatal("Failed to created ap_romfs_embedded.h")

        ctx.env.CXXFLAGS += ['-DHAL_HAVE_AP_ROMFS_EMBEDDED_H']

        # Allow lua to load from ROMFS if any lua files are added
        for file in ctx.env.ROMFS_FILES:
            if file[0].startswith("scripts") and file[0].endswith(".lua"):
                ctx.env.CXXFLAGS += ['-DHAL_HAVE_AP_ROMFS_EMBEDDED_LUA']
                break

Board = BoardMeta('Board', Board.__bases__, dict(Board.__dict__))

def add_dynamic_boards_chibios():
    '''add boards based on existence of hwdef.dat in subdirectories for ChibiOS'''
    add_dynamic_boards_from_hwdef_dir(chibios, 'libraries/AP_HAL_ChibiOS/hwdef')

def add_dynamic_boards_linux():
    '''add boards based on existence of hwdef.dat in subdirectories for '''
    add_dynamic_boards_from_hwdef_dir(LinuxBoard, 'libraries/AP_HAL_Linux/hwdef')

def add_dynamic_boards_from_hwdef_dir(base_type, hwdef_dir):
    '''add boards based on existence of hwdef.dat in subdirectory'''
    dirname, dirlist, filenames = next(os.walk(hwdef_dir))
    for d in dirlist:
        if d in _board_classes.keys():
            continue
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        hwdef_bl = os.path.join(dirname, d, 'hwdef-bl.dat')
        if os.path.exists(hwdef) or os.path.exists(hwdef_bl):
            newclass = type(d, (base_type,), {'name': d})

def add_dynamic_boards_esp32():
    '''add boards based on existence of hwdef.dat in subdirectories for ESP32'''
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ESP32/hwdef'))
    for d in dirlist:
        if d in _board_classes.keys():
            continue
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        if os.path.exists(hwdef):
            mcu_esp32s3 = True if (d[0:7] == "esp32s3") else False
            if mcu_esp32s3:
                newclass = type(d, (esp32s3,), {'name': d})
            else:
                newclass = type(d, (esp32,), {'name': d})

def get_boards_names():
    add_dynamic_boards_chibios()
    add_dynamic_boards_esp32()
    add_dynamic_boards_linux()

    return sorted(list(_board_classes.keys()), key=str.lower)

def is_board_based(board, cls):
    return issubclass(_board_classes[board], cls)

def get_ap_periph_boards():
    '''Add AP_Periph boards based on existence of periph keyword in hwdef.dat or board name'''
    list_ap = [s for s in list(_board_classes.keys()) if "periph" in s]
    dirname, dirlist, filenames = next(os.walk('libraries/AP_HAL_ChibiOS/hwdef'))
    for d in dirlist:
        if d in list_ap:
            continue
        hwdef = os.path.join(dirname, d, 'hwdef.dat')
        if os.path.exists(hwdef):
            ch = chibios_hwdef.ChibiOSHWDef(hwdef=[hwdef], quiet=True)
            try:
                if ch.is_periph_fw_unprocessed():
                    list_ap.append(d)
            except chibios_hwdef.ChibiOSHWDefIncludeNotFoundException as e:
                print(f"{e.includer} includes {e.hwdef} which does not exist")
                sys.exit(1)

    list_ap = list(set(list_ap))
    return list_ap

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
        if ctx.env.BOARD not in boards:
            ctx.fatal("Invalid board '%s': choices are %s" % (ctx.env.BOARD, ', '.join(sorted(boards, key=str.lower))))
        _board = _board_classes[ctx.env.BOARD]()
    return _board

# NOTE: Keeping all the board definitions together so we can easily
# identify opportunities to simplify common flags. In the future might
# be worthy to keep board definitions in files of their own.

class sitl(Board):

    def __init__(self):
        super().__init__()

        self.with_can = True
        self.with_littlefs = True

    def configure_env(self, cfg, env):
        super(sitl, self).configure_env(cfg, env)
        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_SITL',
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_NONE',
            AP_SCRIPTING_CHECKS = 1, # SITL should always do runtime scripting checks
            AP_BARO_PROBE_EXTERNAL_I2C_BUSES = 1,
        )

        env.BOARD_CLASS = "SITL"

        cfg.define('AP_SIM_ENABLED', 1)
        cfg.define('HAL_WITH_SPI', 1)
        cfg.define('HAL_WITH_RAMTRON', 1)
        cfg.define('AP_OPENDRONEID_ENABLED', 1)
        cfg.define('AP_SIGNED_FIRMWARE', 0)

        cfg.define('AP_NOTIFY_LP5562_BUS', 2)
        cfg.define('AP_NOTIFY_LP5562_ADDR', 0x30)

        # turn on fencepoint and rallypoint protocols so they're still tested:
        env.CXXFLAGS.extend([
            '-DAP_MAVLINK_RALLY_POINT_PROTOCOL_ENABLED=HAL_GCS_ENABLED&&HAL_RALLY_ENABLED',
            '-DAC_POLYFENCE_FENCE_POINT_PROTOCOL_SUPPORT=HAL_GCS_ENABLED&&AP_FENCE_ENABLED'
        ])

        try:
            env.CXXFLAGS.remove('-DHAL_NAVEKF2_AVAILABLE=0')
        except ValueError:
            pass
        env.CXXFLAGS += ['-DHAL_NAVEKF2_AVAILABLE=1']

        if self.with_can:
            cfg.define('HAL_NUM_CAN_IFACES', 2)
            env.DEFINES.update(CANARD_MULTI_IFACE=1,
                               CANARD_IFACE_ALL = 0x3,
                               CANARD_ENABLE_CANFD = 1,
                               CANARD_ENABLE_ASSERTS = 1)
            if not cfg.options.force_32bit:
                # needed for cygwin
                env.CXXFLAGS += [ '-DCANARD_64_BIT=1' ]
                env.CFLAGS += [ '-DCANARD_64_BIT=1' ]
            if Utils.unversioned_sys_platform().startswith("linux"):
                cfg.define('HAL_CAN_WITH_SOCKETCAN', 1)
            else:
                cfg.define('HAL_CAN_WITH_SOCKETCAN', 0)

        env.CXXFLAGS += [
            '-Werror=float-equal',
            '-Werror=missing-declarations',
        ]

        if not cfg.options.disable_networking and not 'clang' in cfg.env.COMPILER_CC:
            # lwip doesn't build with clang
            env.CXXFLAGS += ['-DAP_NETWORKING_ENABLED=1']
        
        if cfg.options.ubsan or cfg.options.ubsan_abort:
            env.CXXFLAGS += [
                "-fsanitize=undefined",
                "-fsanitize=float-cast-overflow",
                "-DUBSAN_ENABLED",
            ]
            env.LINKFLAGS += [
                "-fsanitize=undefined",
                "-lubsan",
            ]

        if cfg.options.ubsan_abort:
            env.CXXFLAGS += [
                "-fno-sanitize-recover"
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
            'AP_CSVReader',
        ]

        env.AP_LIBRARIES += [
            'SITL',
        ]

        # wrap malloc to ensure memory is zeroed
        if cfg.env.DEST_OS == 'cygwin':
            pass # handled at runtime in libraries/AP_Common/c++.cpp
        elif platform.system() != 'Darwin':
            env.LINKFLAGS += ['-Wl,--wrap,malloc']
        
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

        if cfg.options.sitl_rgbled:
            env.CXXFLAGS += ['-DWITH_SITL_RGBLED']

        if cfg.options.enable_sfml_audio:
            if not cfg.check_SFML_Audio(env):
                cfg.fatal("Failed to find SFML Audio libraries")
            env.CXXFLAGS += ['-DWITH_SITL_TONEALARM']

        if cfg.options.sitl_littlefs:
            env.CXXFLAGS += ['-DHAL_OS_LITTLEFS_IO=1']

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

        if cfg.options.force_32bit:
            # 32bit platform flags
            env.CXXFLAGS += [
                '-m32',
            ]
            env.CFLAGS += [
                '-m32',
            ]
            env.LDFLAGS += [
                '-m32',
            ]

        # whitelist of compilers which we should build with -Werror
        gcc_whitelist = frozenset([
                ('11','3','0'),
                ('11','4','0'),
                ('12','1','0'),
            ])

        # initialise werr_enabled from defaults:
        werr_enabled = bool('g++' in cfg.env.COMPILER_CXX and cfg.env.CC_VERSION in gcc_whitelist)

        # now process overrides to that default:
        if (cfg.options.Werror is not None and
                cfg.options.Werror == cfg.options.disable_Werror):
            cfg.fatal("Asked to both enable and disable Werror")

        if cfg.options.Werror is not None:
            werr_enabled = cfg.options.Werror
        elif cfg.options.disable_Werror is not None:
            werr_enabled = not cfg.options.disable_Werror

        if werr_enabled:
            cfg.msg("Enabling -Werror", "yes")
            if '-Werror' not in env.CXXFLAGS:
                env.CXXFLAGS += [ '-Werror' ]
        else:
            cfg.msg("Enabling -Werror", "no")
            if '-Werror' in env.CXXFLAGS:
                env.CXXFLAGS.remove('-Werror')

    def get_name(self):
        return self.__class__.__name__


class sitl_periph(sitl):
    def configure_env(self, cfg, env):
        cfg.env.AP_PERIPH = 1
        super(sitl_periph, self).configure_env(cfg, env)
        env.DEFINES.update(
            HAL_BUILD_AP_PERIPH = 1,
            PERIPH_FW = 1,
            HAL_RAM_RESERVE_START = 0,

            CANARD_ENABLE_CANFD = 1,
            CANARD_ENABLE_TAO_OPTION = 1,
            CANARD_MULTI_IFACE = 1,

            # FIXME: SITL library should not be using AP_AHRS:
            AP_AHRS_ENABLED = 1,
            AP_AHRS_BACKEND_DEFAULT_ENABLED = 0,
            AP_AHRS_DCM_ENABLED = 1,  # need a default backend
            AP_EXTERNAL_AHRS_ENABLED = 0,

            HAL_MAVLINK_BINDINGS_ENABLED = 1,

            AP_AIRSPEED_AUTOCAL_ENABLE = 0,
            AP_CAN_SLCAN_ENABLED = 0,
            AP_ICENGINE_ENABLED = 0,
            AP_MISSION_ENABLED = 0,
            AP_RCPROTOCOL_ENABLED = 0,
            AP_RTC_ENABLED = 0,
            AP_SCHEDULER_ENABLED = 0,
            AP_SCRIPTING_ENABLED = 0,
            AP_STATS_ENABLED = 0,
            AP_UART_MONITOR_ENABLED = 1,
            COMPASS_CAL_ENABLED = 0,
            COMPASS_LEARN_ENABLED = 0,
            COMPASS_MOT_ENABLED = 0,
            HAL_CAN_DEFAULT_NODE_ID = 0,
            HAL_CANMANAGER_ENABLED = 0,
            HAL_GCS_ENABLED = 0,
            HAL_GENERATOR_ENABLED = 0,
            HAL_LOGGING_ENABLED = 0,
            HAL_LOGGING_MAVLINK_ENABLED = 0,
            HAL_PROXIMITY_ENABLED = 0,
            HAL_RALLY_ENABLED = 0,
            HAL_SUPPORT_RCOUT_SERIAL = 0,
            AP_TERRAIN_AVAILABLE = 0,
            AP_CUSTOMROTATIONS_ENABLED = 0,
            AP_PERIPH_BATTERY_ENABLED = 0,
            AP_PERIPH_DEVICE_TEMPERATURE_ENABLED = 0,
            AP_PERIPH_SERIAL_OPTIONS_ENABLED = 0,
            AP_PERIPH_ADSB_ENABLED = 0,
            AP_PERIPH_PROXIMITY_ENABLED = 0,
            AP_PERIPH_GPS_ENABLED = 0,
            AP_PERIPH_RELAY_ENABLED = 0,
            AP_PERIPH_IMU_ENABLED = 0,
            AP_PERIPH_MAG_ENABLED = 0,
            AP_PERIPH_BATTERY_BALANCE_ENABLED = 0,
            AP_PERIPH_BATTERY_TAG_ENABLED = 0,
            AP_PERIPH_BATTERY_BMS_ENABLED = 0,
            AP_PERIPH_MSP_ENABLED = 0,
            AP_PERIPH_BARO_ENABLED = 0,
            AP_PERIPH_EFI_ENABLED = 0,
            AP_PERIPH_RANGEFINDER_ENABLED = 0,
            AP_PERIPH_RC_OUT_ENABLED = 0,
            AP_PERIPH_RTC_ENABLED = 0,
            AP_PERIPH_RCIN_ENABLED = 0,
            AP_PERIPH_RPM_ENABLED = 0,
            AP_PERIPH_RPM_STREAM_ENABLED = 0,
            AP_PERIPH_AIRSPEED_ENABLED = 0,
            AP_PERIPH_HOBBYWING_ESC_ENABLED = 0,
            AP_PERIPH_NETWORKING_ENABLED = 0,
            AP_PERIPH_NOTIFY_ENABLED = 0,
            AP_PERIPH_PWM_HARDPOINT_ENABLED = 0,
            AP_PERIPH_ESC_APD_ENABLED = 0,
            AP_PERIPH_NCP5623_LED_WITHOUT_NOTIFY_ENABLED = 0,
            AP_PERIPH_NCP5623_BGR_LED_WITHOUT_NOTIFY_ENABLED = 0,
            AP_PERIPH_TOSHIBA_LED_WITHOUT_NOTIFY_ENABLED = 0,
            AP_PERIPH_BUZZER_ENABLED = 0,
            AP_PERIPH_BUZZER_WITHOUT_NOTIFY_ENABLED = 0,
            AP_PERIPH_RTC_GLOBALTIME_ENABLED = 0,
            AP_PERIPH_ACTUATOR_TELEM_ENABLED = 0,
        )

        try:
            env.CXXFLAGS.remove('-DHAL_NAVEKF2_AVAILABLE=1')
        except ValueError:
            pass
        env.CXXFLAGS += ['-DHAL_NAVEKF2_AVAILABLE=0']

class sitl_periph_universal(sitl_periph):
    def configure_env(self, cfg, env):
        super(sitl_periph_universal, self).configure_env(cfg, env)
        env.DEFINES.update(
            CAN_APP_NODE_NAME = '"org.ardupilot.ap_periph_universal"',
            APJ_BOARD_ID = 100,

            AP_PERIPH_GPS_ENABLED = 1,
            AP_PERIPH_AIRSPEED_ENABLED = 1,
            AP_PERIPH_MAG_ENABLED = 1,
            AP_PERIPH_BARO_ENABLED = 1,
            AP_PERIPH_IMU_ENABLED = 1,
            AP_PERIPH_RANGEFINDER_ENABLED = 1,
            AP_PERIPH_BATTERY_ENABLED = 1,
            AP_PERIPH_EFI_ENABLED = 1,
            AP_PERIPH_RPM_ENABLED = 1,
            AP_PERIPH_RPM_STREAM_ENABLED = 1,
            AP_RPM_STREAM_ENABLED = 1,
            AP_PERIPH_RC_OUT_ENABLED = 1,
            AP_PERIPH_ADSB_ENABLED = 1,
            AP_PERIPH_SERIAL_OPTIONS_ENABLED = 1,
            AP_AIRSPEED_ENABLED = 1,
            AP_BATTERY_ESC_ENABLED = 1,
            HAL_PWM_COUNT = 32,
            HAL_WITH_ESC_TELEM = 1,
            AP_EXTENDED_ESC_TELEM_ENABLED = 1,
            AP_TERRAIN_AVAILABLE = 1,
            HAL_GYROFFT_ENABLED = 0,
        )

class sitl_periph_gps(sitl_periph):
    def configure_env(self, cfg, env):
        cfg.env.AP_PERIPH = 1
        super(sitl_periph_gps, self).configure_env(cfg, env)
        env.DEFINES.update(
            HAL_BUILD_AP_PERIPH = 1,
            PERIPH_FW = 1,
            CAN_APP_NODE_NAME = '"org.ardupilot.ap_periph_gps"',
            APJ_BOARD_ID = 101,

            AP_PERIPH_GPS_ENABLED = 1,
        )

class sitl_periph_battmon(sitl_periph):
    def configure_env(self, cfg, env):
        cfg.env.AP_PERIPH = 1
        super(sitl_periph_battmon, self).configure_env(cfg, env)
        env.DEFINES.update(
            HAL_BUILD_AP_PERIPH = 1,
            PERIPH_FW = 1,
            CAN_APP_NODE_NAME = '"org.ardupilot.ap_periph_battmon"',
            APJ_BOARD_ID = 101,

            AP_PERIPH_BATTERY_ENABLED = 1,
        )

class sitl_periph_battery_tag(sitl_periph):
    def configure_env(self, cfg, env):
        cfg.env.AP_PERIPH = 1
        super(sitl_periph_battery_tag, self).configure_env(cfg, env)
        env.DEFINES.update(
            HAL_BUILD_AP_PERIPH = 1,
            PERIPH_FW = 1,
            CAN_APP_NODE_NAME = '"org.ardupilot.battery_tag"',
            APJ_BOARD_ID = 101,

            AP_SIM_PARAM_ENABLED = 0,
            AP_KDECAN_ENABLED = 0,
            AP_TEMPERATURE_SENSOR_ENABLED = 0,
            AP_PERIPH_BATTERY_TAG_ENABLED = 1,
            AP_RTC_ENABLED = 1,
            AP_PERIPH_RTC_ENABLED = 1,
            AP_PERIPH_RTC_GLOBALTIME_ENABLED = 1,
        )

class sitl_periph_can_to_serial(sitl_periph):
    def configure_env(self, cfg, env):
        cfg.env.AP_PERIPH = 1
        super().configure_env(cfg, env)
        env.DEFINES.update(
            HAL_BUILD_AP_PERIPH = 1,
            PERIPH_FW = 1,
            CAN_APP_NODE_NAME = '"org.ardupilot.serial_passthrough"',
            APJ_BOARD_ID = 101,

        )

class esp32(Board):
    abstract = True
    toolchain = 'xtensa-esp32-elf'

    def configure(self, cfg):
        super(esp32, self).configure(cfg)
        if cfg.env.TOOLCHAIN:
            self.toolchain = cfg.env.TOOLCHAIN
        else:
            # default tool-chain for esp32-based boards:
            self.toolchain = 'xtensa-esp32-elf'

    def configure_env(self, cfg, env):
        env.BOARD_CLASS = "ESP32"

        def expand_path(p):
            print("USING EXPRESSIF IDF:"+str(env.idf))
            return cfg.root.find_dir(env.IDF+p).abspath()
        try:
            env.IDF = os.environ['IDF_PATH'] 
        except:
            env.IDF = cfg.srcnode.abspath()+"/modules/esp_idf"

        super(esp32, self).configure_env(cfg, env)
        cfg.load('esp32')
        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_ESP32',
        )

        tt = self.name[5:] #leave off 'esp32' so we just get 'buzz','diy','icarus, etc

        # this makes sure we get the correct subtype
        env.DEFINES.update(
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_NONE',
        )

        if self.name.endswith("empty"):
            # for empty targets build as SIM-on-HW
            env.DEFINES.update(AP_SIM_ENABLED = 1)
            env.AP_LIBRARIES += [
                'SITL',
            ]
        else:
            env.DEFINES.update(AP_SIM_ENABLED = 0)

        # FreeRTOS component from esp-idf expects this define
        env.DEFINES.update(ESP_PLATFORM = 1)

        env.AP_LIBRARIES += [
            'AP_HAL_ESP32',
        ]

        env.CFLAGS += [
            '-fno-inline-functions',
            '-mlongcalls',
            '-fsingle-precision-constant',
        ]
        env.CFLAGS.remove('-Werror=undef')

        env.CXXFLAGS += ['-mlongcalls',
                         '-Os',
                         '-g',
                         '-ffunction-sections',
                         '-fdata-sections',
                         '-fno-exceptions',
                         '-fno-rtti',
                         '-nostdlib',
                         '-fstrict-volatile-bitfields',
                         '-Wno-sign-compare',
                         '-fno-inline-functions',
                         '-mlongcalls',
                         '-fsingle-precision-constant', # force const vals to be float , not double. so 100.0 means 100.0f 
                         '-fno-threadsafe-statics']
        env.CXXFLAGS.remove('-Werror=undef')
        env.CXXFLAGS.remove('-Werror=shadow')

        # wrap malloc to ensure memory is zeroed
        # note that this also needs to be done in the CMakeLists.txt files
        env.LINKFLAGS += ['-Wl,--wrap,malloc']

        # TODO: remove once hwdef.dat support is in place
        defaults_file = 'libraries/AP_HAL_ESP32/hwdef/%s/defaults.parm' % self.get_name()
        if os.path.exists(defaults_file):
            env.ROMFS_FILES += [('defaults.parm', defaults_file)]
            env.DEFINES.update(
                HAL_PARAM_DEFAULTS_PATH='"@ROMFS/defaults.parm"',
            )

        env.AP_PROGRAM_AS_STLIB = True
        #if cfg.options.enable_profile:
        #    env.CXXFLAGS += ['-pg',
        #                     '-DENABLE_PROFILE=1']
    def pre_build(self, bld):
        '''pre-build hook that gets called before dynamic sources'''
        from waflib.Context import load_tool
        module = load_tool('esp32', [], with_sys_path=True)
        fun = getattr(module, 'pre_build', None)
        if fun:
            fun(bld)
        super(esp32, self).pre_build(bld)


    def build(self, bld):
        super(esp32, self).build(bld)
        bld.load('esp32')

    def get_name(self):
        return self.__class__.__name__

class esp32s3(esp32):
    abstract = True
    toolchain = 'xtensa-esp32s3-elf'

    def configure_env(self, cfg, env):
        if cfg.env.TOOLCHAIN:
            self.toolchain = cfg.env.TOOLCHAIN
        else:
            # default tool-chain for esp32-based boards:
            self.toolchain = 'xtensa-esp32s3-elf'

        if hasattr(self, 'hwdef'):
            cfg.env.HWDEF = self.hwdef
        super(esp32s3, self).configure_env(cfg, env)

class chibios(Board):
    abstract = True
    toolchain = 'arm-none-eabi'

    def configure_env(self, cfg, env):
        if hasattr(self, 'hwdef'):
            cfg.env.HWDEF = self.hwdef
        super(chibios, self).configure_env(cfg, env)

        cfg.load('chibios')
        env.BOARD = self.name
        env.BOARD_CLASS = "ChibiOS"

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_CHIBIOS',
            HAVE_STD_NULLPTR_T = 0,
        )

        env.AP_LIBRARIES += [
            'AP_HAL_ChibiOS',
        ]

        # make board name available for USB IDs
        env.CHIBIOS_BOARD_NAME = 'HAL_BOARD_NAME="%s"' % self.name
        env.HAL_MAX_STACK_FRAME_SIZE = 'HAL_MAX_STACK_FRAME_SIZE=%d' % 1300 # set per Wframe-larger-than, ensure its same
        env.CFLAGS += cfg.env.CPU_FLAGS + [
            '-Wlogical-op',
            '-Wframe-larger-than=1300',
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
            '-fno-math-errno',
            '-mno-thumb-interwork',
            '-mthumb',
            '--specs=nano.specs',
            '--specs=nosys.specs',
            '-D__USE_CMSIS',
            '-Werror=deprecated-declarations',
            '-DNDEBUG=1'
        ]
        if not cfg.options.Werror:
            env.CFLAGS += [
            '-Wno-error=double-promotion',
            '-Wno-error=missing-declarations',
            '-Wno-error=float-equal',
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
            '--specs=nano.specs',
            '--specs=nosys.specs',
            '-L%s' % env.BUILDROOT,
            '-L%s' % cfg.srcnode.make_node('modules/ChibiOS/os/common/startup/ARMCMx/compilers/GCC/ld/').abspath(),
            '-L%s' % cfg.srcnode.make_node('libraries/AP_HAL_ChibiOS/hwdef/common/').abspath(),
            '-Wl,-Map,Linker.map,%s--cref,--gc-sections,--no-warn-mismatch,--library-path=/ld,--script=ldscript.ld,--defsym=__process_stack_size__=%s,--defsym=__main_stack_size__=%s' % ("--print-memory-usage," if cfg.env.EXT_FLASH_SIZE_MB > 0 and cfg.env.INT_FLASH_PRIMARY == 0 else "", cfg.env.PROCESS_STACK, cfg.env.MAIN_STACK)
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

        if cfg.env.COMPILER_CXX == "g++":
            if not self.cc_version_gte(cfg, 10, 2):
                # require at least 10.2 compiler
                cfg.fatal("ChibiOS build requires g++ version 10.2.1 or later, found %s" % '.'.join(cfg.env.CC_VERSION))
            
        if cfg.env.ENABLE_ASSERTS:
            cfg.msg("Enabling ChibiOS asserts", "yes")
            env.CFLAGS += [ '-DHAL_CHIBIOS_ENABLE_ASSERTS' ]
            env.CXXFLAGS += [ '-DHAL_CHIBIOS_ENABLE_ASSERTS' ]
        else:
            cfg.msg("Enabling ChibiOS asserts", "no")


        if cfg.env.SAVE_TEMPS:
            env.CXXFLAGS += [ '-S', '-save-temps=obj' ]

        if cfg.options.disable_watchdog:
            cfg.msg("Disabling Watchdog", "yes")
            env.CFLAGS += [ '-DDISABLE_WATCHDOG' ]
            env.CXXFLAGS += [ '-DDISABLE_WATCHDOG' ]
        else:
            cfg.msg("Disabling Watchdog", "no")

        if cfg.env.ENABLE_MALLOC_GUARD:
            cfg.msg("Enabling malloc guard", "yes")
            env.CFLAGS += [ '-DHAL_CHIBIOS_ENABLE_MALLOC_GUARD' ]
            env.CXXFLAGS += [ '-DHAL_CHIBIOS_ENABLE_MALLOC_GUARD' ]
        else:
            cfg.msg("Enabling malloc guard", "no")
            
        if cfg.env.ENABLE_STATS:
            cfg.msg("Enabling ChibiOS thread statistics", "yes")
            env.CFLAGS += [ '-DHAL_ENABLE_THREAD_STATISTICS' ]
            env.CXXFLAGS += [ '-DHAL_ENABLE_THREAD_STATISTICS' ]
        else:
            cfg.msg("Enabling ChibiOS thread statistics", "no")

        if cfg.env.SIM_ENABLED:
            env.DEFINES.update(
                AP_SIM_ENABLED = 1,
            )
            env.AP_LIBRARIES += [
                'SITL',
            ]
        else:
            env.DEFINES.update(
                AP_SIM_ENABLED = 0,
            )

        env.LIB += ['gcc', 'm']

        env.GIT_SUBMODULES += [
            'ChibiOS',
        ]

        env.INCLUDES += [
            cfg.srcnode.find_dir('libraries/AP_GyroFFT/CMSIS_5/include').abspath(),
            cfg.srcnode.find_dir('modules/lwip/src/include/compat/posix').abspath()
        ]

        # whitelist of compilers which we should build with -Werror
        gcc_whitelist = frozenset([
            ('4','9','3'),
            ('6','3','1'),
            ('9','2','1'),
            ('9','3','1'),
            ('10','2','1'),
            ('11','3','0'),
            ('11','4','0'),
        ])

        if cfg.env.HAL_CANFD_SUPPORTED:
            env.DEFINES.update(CANARD_ENABLE_CANFD=1)
        else:
            env.DEFINES.update(CANARD_ENABLE_TAO_OPTION=1)
        if not cfg.options.bootloader and cfg.env.HAL_NUM_CAN_IFACES:
            if int(cfg.env.HAL_NUM_CAN_IFACES) >= 1:
                env.DEFINES.update(CANARD_IFACE_ALL=(1<<int(cfg.env.HAL_NUM_CAN_IFACES))-1)
        if cfg.options.Werror or cfg.env.CC_VERSION in gcc_whitelist:
            cfg.msg("Enabling -Werror", "yes")
            if '-Werror' not in env.CXXFLAGS:
                env.CXXFLAGS += [ '-Werror' ]
        else:
            cfg.msg("Enabling -Werror", "no")

        if cfg.options.signed_fw:
            cfg.define('AP_SIGNED_FIRMWARE', 1)
            env.CFLAGS += [
                '-DAP_SIGNED_FIRMWARE=1',
            ]
        else:
            cfg.define('AP_SIGNED_FIRMWARE', 0)
            env.CFLAGS += [
                '-DAP_SIGNED_FIRMWARE=0',
            ]

        try:
            import intelhex
            env.HAVE_INTEL_HEX = True
            cfg.msg("Checking for intelhex module:", 'OK')
        except Exception:
            cfg.msg("Checking for intelhex module:", 'disabled', color='YELLOW')
            env.HAVE_INTEL_HEX = False

        if cfg.options.enable_new_checking:
            env.CHECK_SYMBOLS = True
        else:
            # disable new checking on ChibiOS by default to save flash
            # we enable it in a CI test to catch incorrect usage
            env.CXXFLAGS += [
                "-DNEW_NOTHROW=new",
                "-fcheck-new", # rely on -fcheck-new ensuring nullptr checks
                ]

    def build(self, bld):
        super(chibios, self).build(bld)
        bld.ap_version_append_str('CHIBIOS_GIT_VERSION', bld.git_submodule_head_hash('ChibiOS', short=True), "12345678")
        bld.load('chibios')

    def pre_build(self, bld):
        '''pre-build hook that gets called before dynamic sources'''
        from waflib.Context import load_tool
        module = load_tool('chibios', [], with_sys_path=True)
        fun = getattr(module, 'pre_build', None)
        if fun:
            fun(bld)
        super(chibios, self).pre_build(bld)

    def get_name(self):
        return self.name

class LinuxBoard(Board):
    '''an abstract base class for Linux boards to inherit from'''
    abstract = True

    def __init__(self):
        super().__init__()

        self.with_can = True

    def configure(self, cfg):
        if hasattr(self, 'hwdef'):
            cfg.env.HWDEF = self.hwdef

        # load hwdef, extract toolchain from cfg.env:
        cfg.load('linux')
        if cfg.env.TOOLCHAIN:
            self.toolchain = cfg.env.TOOLCHAIN
        else:
            # default tool-chain for Linux-based boards:
            self.toolchain = 'arm-linux-gnueabihf'

        # we should be able to do better here:
        if cfg.env.WITH_CAN:
            self.with_can = True

        super().configure(cfg)

    def configure_env(self, cfg, env):
        super().configure_env(cfg, env)

        env.BOARD_CLASS = "LINUX"

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_LINUX',
            AP_SIM_ENABLED = 0,
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

        # wrap malloc to ensure memory is zeroed
        env.LINKFLAGS += ['-Wl,--wrap,malloc']

        if cfg.options.force_32bit:
            env.DEFINES.update(
                HAL_FORCE_32BIT = 1,
            )
            # 32bit platform flags
            cfg.env.CXXFLAGS += [
                '-m32',
            ]
            cfg.env.CFLAGS += [
                '-m32',
            ]
            cfg.env.LDFLAGS += [
                '-m32',
            ]
        else:
            env.DEFINES.update(
                HAL_FORCE_32BIT = 0,
            )

        if self.with_can:
            env.DEFINES.update(CANARD_MULTI_IFACE=1,
                               CANARD_IFACE_ALL = 0x3)

        if cfg.options.apstatedir:
            cfg.define('AP_STATEDIR', cfg.options.apstatedir)

        defaults_file = 'libraries/AP_HAL_Linux/boards/%s/defaults.parm' % self.get_name()
        if os.path.exists(defaults_file):
            env.ROMFS_FILES += [('defaults.parm', defaults_file)]
            env.DEFINES.update(
                HAL_PARAM_DEFAULTS_PATH='"@ROMFS/defaults.parm"',
            )

    def pre_build(self, bld):
        '''pre-build hook that gets called before dynamic sources'''
        from waflib.Context import load_tool
        module = load_tool('linux', [], with_sys_path=True)
        fun = getattr(module, 'pre_build', None)
        if fun:
            fun(bld)
        super().pre_build(bld)

    def build(self, bld):
        super().build(bld)
        bld.load('linux')
        if bld.options.upload:
            waflib.Options.commands.append('rsync')
            # Avoid infinite recursion
            bld.options.upload = False

    def get_name(self):
        # get name of class
        return self.__class__.__name__

class SITL_static(sitl):
    def configure_env(self, cfg, env):
        super(SITL_static, self).configure_env(cfg, env)
        cfg.env.STATIC_LINKING = True

class SITL_x86_64_linux_gnu(SITL_static):
    toolchain = 'x86_64-linux-gnu'

class SITL_arm_linux_gnueabihf(SITL_static):
    toolchain = 'arm-linux-gnueabihf'

class QURT(Board):
    '''support for QURT based boards'''
    toolchain = 'custom'

    def __init__(self):
        super().__init__()

        self.with_can = False

    def configure_toolchain(self, cfg):
        cfg.env.CXX_NAME = 'gcc'
        cfg.env.HEXAGON_SDK_DIR = "/opt/hexagon-sdk/4.1.0.4-lite"
        cfg.env.CC_VERSION = ('4','1','0')
        cfg.env.TOOLCHAIN_DIR = cfg.env.HEXAGON_SDK_DIR + "/tools/HEXAGON_Tools/8.4.05/Tools"
        cfg.env.COMPILER_CC = cfg.env.TOOLCHAIN_DIR + "/bin/hexagon-clang"
        cfg.env.COMPILER_CXX = cfg.env.TOOLCHAIN_DIR + "/bin/hexagon-clang++"
        cfg.env.LINK_CXX = cfg.env.HEXAGON_SDK_DIR + "/tools/HEXAGON_Tools/8.4.05/Tools/bin/hexagon-link"
        cfg.env.CXX = ["ccache", cfg.env.COMPILER_CXX]
        cfg.env.CC = ["ccache", cfg.env.COMPILER_CC]
        cfg.env.CXX_TGT_F = ['-c', '-o']
        cfg.env.CC_TGT_F = ['-c', '-o']
        cfg.env.CCLNK_SRC_F = []
        cfg.env.CXXLNK_SRC_F = []
        cfg.env.CXXLNK_TGT_F = ['-o']
        cfg.env.CCLNK_TGT_F = ['-o']
        cfg.env.CPPPATH_ST = '-I%s'
        cfg.env.DEFINES_ST = '-D%s'
        cfg.env.AR = cfg.env.HEXAGON_SDK_DIR + "/tools/HEXAGON_Tools/8.4.05/Tools/bin/hexagon-ar"
        cfg.env.ARFLAGS = 'rcs'
        cfg.env.cxxstlib_PATTERN = 'lib%s.a'
        cfg.env.cstlib_PATTERN = 'lib%s.a'
        cfg.env.LIBPATH_ST = '-L%s'
        cfg.env.LIB_ST = '-l%s'
        cfg.env.SHLIB_MARKER = ''
        cfg.env.STLIBPATH_ST = '-L%s'
        cfg.env.STLIB_MARKER = ''
        cfg.env.STLIB_ST = '-l%s'
        cfg.env.LDFLAGS = [
            '-lgcc',
            cfg.env.TOOLCHAIN_DIR + '/target/hexagon/lib/v66/G0/pic/finiS.o'
        ]

    def configure_env(self, cfg, env):
        super(QURT, self).configure_env(cfg, env)

        env.BOARD_CLASS = "QURT"
        env.HEXAGON_APP = "libardupilot.so"
        env.INCLUDES += [cfg.env.HEXAGON_SDK_DIR + "/rtos/qurt/computev66/include/qurt"]
        env.INCLUDES += [cfg.env.HEXAGON_SDK_DIR + "/rtos/qurt/computev66/include/posix"]

        CFLAGS = "-MD -mv66 -fPIC -mcpu=hexagonv66 -G0 -fdata-sections -ffunction-sections -fomit-frame-pointer -fmerge-all-constants -fno-signed-zeros -fno-trapping-math -freciprocal-math -fno-math-errno -fno-strict-aliasing -fvisibility=hidden -fno-rtti -fmath-errno"
        env.CXXFLAGS += CFLAGS.split()
        env.CFLAGS += CFLAGS.split()

        env.DEFINES.update(
            CONFIG_HAL_BOARD = 'HAL_BOARD_QURT',
            CONFIG_HAL_BOARD_SUBTYPE = 'HAL_BOARD_SUBTYPE_NONE',
            AP_SIM_ENABLED = 0,
        )

        env.LINKFLAGS = [
            "-march=hexagon",
            "-mcpu=hexagonv66",
            "-shared",
            "-call_shared",
            "-G0",
            cfg.env.TOOLCHAIN_DIR + "/target/hexagon/lib/v66/G0/pic/initS.o",
            f"-L{cfg.env.TOOLCHAIN_DIR}/target/hexagon/lib/v66/G0/pic",
            "-Bsymbolic",
            cfg.env.TOOLCHAIN_DIR + "/target/hexagon/lib/v66/G0/pic/libgcc.a",
            "--wrap=malloc",
            "--wrap=calloc",
            "--wrap=free",
            "--wrap=printf",
            "--wrap=strdup",
            "--wrap=__stack_chk_fail",
            "-lc"
        ]

        if not cfg.env.DEBUG:
            env.CXXFLAGS += [
                '-O3',
            ]

        env.AP_LIBRARIES += [
            'AP_HAL_QURT',
        ]

    def build(self, bld):
        super(QURT, self).build(bld)
        bld.load('qurt')

    def get_name(self):
        # get name of class
        return self.__class__.__name__
    

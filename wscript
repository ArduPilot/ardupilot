#!/usr/bin/env python
# encoding: utf-8

import os.path
import sys
sys.path.insert(0, 'Tools/ardupilotwaf/')

import ardupilotwaf
import waflib

# TODO: implement a command 'waf help' that shows the basic tasks a
# developer might want to do: e.g. how to configure a board, compile a
# vehicle, compile all the examples, add a new example. Should fit in
# less than a terminal screen, ideally commands should be copy
# pastable. Add the 'export waf="$PWD/waf"' trick to be copy-pastable
# as well.

# TODO: add support for unit tests.

# TODO: replace defines with the use of a generated config.h file
# this makes recompilation at least when defines change. which might
# be sufficient.

# TODO: set git version as part of build preparation.

# TODO: Check if we should simply use the signed 'waf' "binary" (after
# verifying it) instead of generating it ourselves from the sources.

# TODO: evaluate if we need shortcut commands for the common targets
# (vehicles). currently using waf --targets=NAME the target name must
# contain the board extension so make it less convenient, maybe hook
# to support automatic filling this extension?

PROJECT_CONFIG = dict(
    CFLAGS=[
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
    ],

    CXXFLAGS=[
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
    ],

    LINKFLAGS=[
        '-Wl,--gc-sections',
    ],
)

# NOTE: Keeping all the board definitions together so we can easily
# identify opportunities to simplify how it works. In the future might
# be worthy to keep board definitions in files of their own.
BOARDS = {
    'sitl': dict(
        DEFINES=[
            'CONFIG_HAL_BOARD=HAL_BOARD_SITL',
            'CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE',
        ],

        LIB=[
            'm',
            'pthread',
        ],

        AP_LIBRARIES=[
            'AP_HAL_SITL',
            'SITL',
        ],
    ),

    'linux': dict(
        DEFINES=[
            'CONFIG_HAL_BOARD=HAL_BOARD_LINUX',
            'CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_LINUX_NONE',
        ],

        LIB=[
            'm',
            'pthread',
            'rt',
        ],

        AP_LIBRARIES=[
            'AP_HAL_Linux',
        ],
    ),

    'minlure': dict(
        DEFINES=[
            'CONFIG_HAL_BOARD=HAL_BOARD_LINUX',
            'CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_LINUX_MINLURE',
        ],

        LIB=[
            'm',
            'pthread',
            'rt',
        ],

        AP_LIBRARIES=[
            'AP_HAL_Linux',
        ],
    ),
}

BOARDS_NAMES = sorted(list(BOARDS.keys()))

def options(opt):
    opt.load('compiler_cxx compiler_c waf_unit_test')
    opt.add_option('--board',
                   action='store',
                   choices=BOARDS_NAMES,
                   default='sitl',
                   help='Target board to build, choices are %s' % BOARDS_NAMES)

    g = opt.add_option_group('Check options')
    g.add_option('--check-verbose',
                 action='store_true',
                 help='Output all test programs')

def configure(cfg):
    cfg.load('compiler_cxx compiler_c')
    cfg.load('clang_compilation_database')
    cfg.load('waf_unit_test')

    cfg.env.HAS_GTEST = cfg.check_cxx(
        lib='gtest',
        mandatory=False,
        uselib_store='GTEST',
        errmsg='not found, unit tests disabled',
    )

    cfg.msg('Setting board to', cfg.options.board)
    cfg.env.BOARD = cfg.options.board
    board = BOARDS[cfg.env.BOARD]

    # Always prepend so that arguments passed in the command line get the
    # priority. Board configuration gets priority over the project
    # configuration.
    for k in board.keys():
        cfg.env.prepend_value(k, board[k])
    for k in PROJECT_CONFIG.keys():
        cfg.env.prepend_value(k, PROJECT_CONFIG[k])

    cfg.env.prepend_value('INCLUDES', [
        cfg.srcnode.abspath() + '/libraries/'
    ])

    # TODO: Investigate if code could be changed to not depend on the
    # source absolute path.
    cfg.env.prepend_value('DEFINES', [
        'SKETCHBOOK="' + cfg.srcnode.abspath() + '"',
    ])

def collect_dirs_to_recurse(bld, globs, **kw):
    dirs = []
    globs = waflib.Utils.to_list(globs)
    for g in globs:
        for d in bld.srcnode.ant_glob(g + '/wscript', **kw):
            dirs.append(d.parent.relpath())
    return dirs

def build(bld):
    # NOTE: Static library with vehicle set to UNKNOWN, shared by all
    # the tools and examples. This is the first step until the
    # dependency on the vehicles is reduced. Later we may consider
    # split into smaller pieces with well defined boundaries.
    ardupilotwaf.vehicle_stlib(
        bld,
        name='ap',
        vehicle='UNKNOWN',
        libraries=ardupilotwaf.get_all_libraries(bld),
    )

    # TODO: Currently each vehicle also generate its own copy of the
    # libraries. Fix this, or at least reduce the amount of
    # vehicle-dependent libraries.
    vehicles = collect_dirs_to_recurse(bld, '*')

    # NOTE: we need to sort to ensure the repeated sources get the
    # same index, and random ordering of the filesystem doesn't cause
    # recompilation.
    vehicles.sort()

    tools = collect_dirs_to_recurse(bld, 'Tools/*')
    examples = collect_dirs_to_recurse(bld,
                                       'libraries/*/examples/*',
                                       excl='libraries/AP_HAL_* libraries/SITL')

    tests = collect_dirs_to_recurse(bld,
                                    '**/tests',
                                    excl='modules Tools libraries/AP_HAL_* libraries/SITL')
    board_tests = ['libraries/%s/**/tests' % l for l in bld.env.AP_LIBRARIES]
    tests.extend(collect_dirs_to_recurse(bld, board_tests))

    hal_examples = []
    for l in bld.env.AP_LIBRARIES:
        hal_examples.extend(collect_dirs_to_recurse(bld, 'libraries/' + l + '/examples/*'))

    for d in vehicles + tools + examples + hal_examples + tests:
        bld.recurse(d)

    if bld.cmd == 'check':
        if not bld.env.HAS_GTEST:
            bld.fatal('check: gtest library is required')
        bld.add_post_fun(ardupilotwaf.test_summary)

class CheckContext(waflib.Build.BuildContext):
    '''executes tests after build'''
    cmd = 'check'

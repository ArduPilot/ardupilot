#!/usr/bin/env python
# encoding: utf-8

from __future__ import print_function

import os.path
import sys
sys.path.insert(0, 'Tools/ardupilotwaf/')

import ardupilotwaf
import boards

from waflib import ConfigSet, Utils
from waflib.Build import BuildContext, CleanContext, InstallContext, UninstallContext

# TODO: implement a command 'waf help' that shows the basic tasks a
# developer might want to do: e.g. how to configure a board, compile a
# vehicle, compile all the examples, add a new example. Should fit in
# less than a terminal screen, ideally commands should be copy
# pastable. Add the 'export waf="$PWD/waf"' trick to be copy-pastable
# as well.

# TODO: replace defines with the use of a generated config.h file
# this makes recompilation at least when defines change. which might
# be sufficient.

# TODO: set git version as part of build preparation.

def init(ctx):
    env = ConfigSet.ConfigSet()
    try:
        env.load('build/c4che/_cache.py')
    except:
        return

    # define the variant build commands according to the board
    for c in (BuildContext, CleanContext, InstallContext, UninstallContext, CheckContext):
        class context(c):
            variant = env.BOARD

def options(opt):
    boards_names = boards.get_boards_names()

    opt.load('compiler_cxx compiler_c waf_unit_test')
    opt.add_option('--board',
                   action='store',
                   choices=boards_names,
                   default='sitl',
                   help='Target board to build, choices are %s' % boards_names)

    g = opt.add_option_group('Check options')
    g.add_option('--check-verbose',
                 action='store_true',
                 help='Output all test programs')

def configure(cfg):
    cfg.env.BOARD = cfg.options.board
    # use a different variant for each board
    cfg.setenv(cfg.env.BOARD)

    cfg.msg('Setting board to', cfg.options.board)
    cfg.env.BOARD = cfg.options.board
    board_dict = boards.BOARDS[cfg.env.BOARD].get_merged_dict()

    # Always prepend so that arguments passed in the command line get
    # the priority.
    for k in board_dict:
        val = board_dict[k]
        # Dictionaries (like 'DEFINES') are converted to lists to
        # conform to waf conventions.
        if isinstance(val, dict):
            for item in val.items():
                cfg.env.prepend_value(k, '%s=%s' % item)
        else:
            cfg.env.prepend_value(k, val)

    cfg.load('toolchain')
    cfg.load('compiler_cxx compiler_c')
    cfg.load('clang_compilation_database')
    cfg.load('waf_unit_test')
    cfg.load('gbenchmark')
    cfg.load('static_linking')

    cfg.start_msg('Benchmarks')
    if cfg.env.HAS_GBENCHMARK:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.env.HAS_GTEST = cfg.check_cxx(
        lib='gtest',
        mandatory=False,
        uselib_store='GTEST',
        errmsg='not found, unit tests disabled',
    )

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
    globs = Utils.to_list(globs)
    for g in globs:
        for d in bld.srcnode.ant_glob(g + '/wscript', **kw):
            dirs.append(d.parent.relpath())
    return dirs

def list_boards(ctx):
    print(*boards.get_boards_names())

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

    benchmarks = collect_dirs_to_recurse(bld,
                                         '**/benchmarks',
                                         excl='modules Tools libraries/AP_HAL_* libraries/SITL')
    board_benchmarks = ['libraries/%s/**/benchmarks' % l for l in bld.env.AP_LIBRARIES]
    benchmarks.extend(collect_dirs_to_recurse(bld, board_benchmarks))

    hal_examples = []
    for l in bld.env.AP_LIBRARIES:
        hal_examples.extend(collect_dirs_to_recurse(bld, 'libraries/' + l + '/examples/*'))

    for d in vehicles + tools + examples + hal_examples + tests + benchmarks:
        bld.recurse(d)

    if bld.cmd == 'check':
        if not bld.env.HAS_GTEST:
            bld.fatal('check: gtest library is required')
        bld.add_post_fun(ardupilotwaf.test_summary)

class CheckContext(BuildContext):
    '''executes tests after build'''
    cmd = 'check'

#!/usr/bin/env python
# encoding: utf-8

from __future__ import print_function

import os.path
import sys
sys.path.insert(0, 'Tools/ardupilotwaf/')

import ardupilotwaf
import boards

from waflib import Build, ConfigSet, Context, Utils

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
    for c in Context.classes:
        if not issubclass(c, Build.BuildContext):
            continue
        c.variant = env.BOARD

def options(opt):
    opt.load('ardupilotwaf')
    boards_names = boards.get_boards_names()

    opt.load('compiler_cxx compiler_c waf_unit_test python')
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
    boards.get_board(cfg.env.BOARD).configure(cfg)

    cfg.load('toolchain')
    cfg.load('compiler_cxx compiler_c')
    cfg.load('clang_compilation_database')
    cfg.load('waf_unit_test')
    cfg.load('mavgen')
    cfg.load('git_submodule')
    cfg.load('gbenchmark')
    cfg.load('gtest')
    cfg.load('static_linking')

    cfg.start_msg('Benchmarks')
    if cfg.env.HAS_GBENCHMARK:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Unit tests')
    if cfg.env.HAS_GTEST:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.env.append_value('GIT_SUBMODULES', 'mavlink')

    cfg.env.prepend_value('INCLUDES', [
        cfg.srcnode.abspath() + '/libraries/',
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

def _build_cmd_tweaks(bld):
    if bld.cmd == 'check-all':
        bld.options.all_tests = True
        bld.cmd = 'check'

    if bld.cmd == 'check':
        bld.options.clear_failed_tests = True
        if not bld.env.HAS_GTEST:
            bld.fatal('check: gtest library is required')
        bld.add_post_fun(ardupilotwaf.test_summary)

def _build_dynamic_sources(bld):
    bld(
        features='mavgen',
        source='modules/mavlink/message_definitions/v1.0/ardupilotmega.xml',
        output_dir='libraries/GCS_MAVLink/include/mavlink/v1.0/',
        name='mavlink',
        # this below is not ideal, mavgen tool should set this, but that's not
        # currently possible
        export_includes=[
            bld.bldnode.make_node('libraries').abspath(),
            bld.bldnode.make_node('libraries/GCS_MAVLink').abspath(),
        ],
    )

def _build_common_taskgens(bld):
    # NOTE: Static library with vehicle set to UNKNOWN, shared by all
    # the tools and examples. This is the first step until the
    # dependency on the vehicles is reduced. Later we may consider
    # split into smaller pieces with well defined boundaries.
    bld.ap_stlib(
        name='ap',
        vehicle='UNKNOWN',
        libraries=bld.ap_get_all_libraries(),
        use='mavlink',
    )

    bld.libgtest()

def _build_recursion(bld):
    common_dirs_patterns = [
        # TODO: Currently each vehicle also generate its own copy of the
        # libraries. Fix this, or at least reduce the amount of
        # vehicle-dependent libraries.
        '*',
        'Tools/*',
        'libraries/*/examples/*',
        '**/tests',
        '**/benchmarks',
    ]

    common_dirs_excl = [
        'modules',
        'libraries/AP_HAL_*',
        'libraries/SITL',
    ]

    hal_dirs_patterns = [
        'libraries/%s/**/tests',
        'libraries/%s/**/benchmarks',
        'libraries/%s/examples/*',
    ]

    dirs_to_recurse = collect_dirs_to_recurse(
        bld,
        common_dirs_patterns,
        excl=common_dirs_excl,
    )

    for p in hal_dirs_patterns:
        dirs_to_recurse += collect_dirs_to_recurse(
            bld,
            [p % l for l in bld.env.AP_LIBRARIES],
        )

    # NOTE: we need to sort to ensure the repeated sources get the
    # same index, and random ordering of the filesystem doesn't cause
    # recompilation.
    dirs_to_recurse.sort()

    for d in dirs_to_recurse:
        bld.recurse(d)

def build(bld):
    bld.post_mode = Build.POST_LAZY

    bld.load('ardupilotwaf')

    _build_cmd_tweaks(bld)

    bld.add_group('git_submodules')
    for name in bld.env.GIT_SUBMODULES:
        bld.git_submodule(name)

    bld.add_group('dynamic_sources')
    _build_dynamic_sources(bld)

    bld.add_group('build')
    _build_common_taskgens(bld)

    _build_recursion(bld)

ardupilotwaf.build_command('check',
    program_group_list='all',
    doc='builds all programs and run tests',
)
ardupilotwaf.build_command('check-all',
    program_group_list='all',
    doc='shortcut for `waf check --alltests`',
)

ardupilotwaf.build_command('copter',
    targets='bin/arducopter',
    doc='builds arducopter',
)
ardupilotwaf.build_command('plane',
    targets='bin/arduplane',
    doc='builds arduplane',
)
ardupilotwaf.build_command('rover',
    targets='bin/ardurover',
    doc='builds ardurover',
)

for program_group in ('all', 'bin', 'tools', 'examples', 'tests', 'benchmarks'):
    ardupilotwaf.build_command(program_group,
        program_group_list=program_group,
        doc='builds all programs of %s group' % program_group,
    )

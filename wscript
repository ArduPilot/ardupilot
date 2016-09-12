#!/usr/bin/env python
# encoding: utf-8

from __future__ import print_function

import os.path
import sys
sys.path.insert(0, 'Tools/ardupilotwaf/')

import ardupilotwaf
import boards

from waflib import Build, ConfigSet, Configure, Context, Utils

# TODO: implement a command 'waf help' that shows the basic tasks a
# developer might want to do: e.g. how to configure a board, compile a
# vehicle, compile all the examples, add a new example. Should fit in
# less than a terminal screen, ideally commands should be copy
# pastable. Add the 'export waf="$PWD/waf"' trick to be copy-pastable
# as well.

# TODO: replace defines with the use of the generated ap_config.h file
# this makes recompilation at least when defines change. which might
# be sufficient.

def _set_build_context_variant(variant):
    for c in Context.classes:
        if not issubclass(c, Build.BuildContext):
            continue
        c.variant = variant

def init(ctx):
    env = ConfigSet.ConfigSet()
    try:
        p = os.path.join(Context.out_dir, Build.CACHE_DIR, Build.CACHE_SUFFIX)
        env.load(p)
    except:
        return

    Configure.autoconfig = 'clobber' if env.AUTOCONFIG else False

    if 'VARIANT' not in env:
        return

    # define the variant build commands according to the board
    _set_build_context_variant(env.VARIANT)

def options(opt):
    opt.load('compiler_cxx compiler_c waf_unit_test python')
    opt.load('ardupilotwaf')
    opt.load('build_summary')

    g = opt.ap_groups['configure']

    boards_names = boards.get_boards_names()
    g.add_option('--board',
        action='store',
        choices=boards_names,
        default='sitl',
        help='Target board to build, choices are %s.' % boards_names)

    g.add_option('--debug',
        action='store_true',
        default=False,
        help='Configure as debug variant.')

    g.add_option('--no-autoconfig',
        dest='autoconfig',
        action='store_false',
        default=True,
        help='''
Disable autoconfiguration feature. By default, the build system triggers a
reconfiguration whenever it thinks it's necessary - this option disables that.
''')

    g.add_option('--no-submodule-update',
        dest='submodule_update',
        action='store_false',
        default=True,
        help='''
Don't update git submodules. Useful for building with submodules at specific
revisions.
''')

    g.add_option('--enable-benchmarks',
        action='store_true',
        default=False,
        help='Enable benchmarks.')

    g.add_option('--disable-lttng', action='store_true',
        default=False,
        help="Don't use lttng even if supported by board and dependencies available")

    g.add_option('--disable-libiio', action='store_true',
        default=False,
        help="Don't use libiio even if supported by board and dependencies available")

    g.add_option('--disable-tests', action='store_true',
        default=False,
        help="Disable compilation and test execution")

    g.add_option('--static',
        action='store_true',
        default=False,
        help='Force a static build')

def _collect_autoconfig_files(cfg):
    for m in sys.modules.values():
        paths = []
        if hasattr(m, '__file__'):
            paths.append(m.__file__)
        elif hasattr(m, '__path__'):
            for p in m.__path__:
                paths.append(p)

        for p in paths:
            if p in cfg.files or not os.path.isfile(p):
                continue

            with open(p, 'rb') as f:
                cfg.hash = Utils.h_list((cfg.hash, f.read()))
                cfg.files.append(p)

def configure(cfg):
    cfg.env.BOARD = cfg.options.board
    cfg.env.DEBUG = cfg.options.debug
    cfg.env.AUTOCONFIG = cfg.options.autoconfig

    cfg.env.VARIANT = cfg.env.BOARD
    if cfg.env.DEBUG:
        cfg.env.VARIANT += '-debug'

    _set_build_context_variant(cfg.env.VARIANT)
    cfg.setenv(cfg.env.VARIANT)

    cfg.env.BOARD = cfg.options.board
    cfg.env.DEBUG = cfg.options.debug

    # Allow to differentiate our build from the make build
    cfg.define('WAF_BUILD', 1)

    cfg.msg('Autoconfiguration', 'enabled' if cfg.options.autoconfig else 'disabled')

    if cfg.options.static:
        cfg.msg('Using static linking', 'yes', color='YELLOW')
        cfg.env.STATIC_LINKING = True

    cfg.load('ap_library')

    cfg.msg('Setting board to', cfg.options.board)
    cfg.get_board().configure(cfg)

    cfg.load('clang_compilation_database')
    cfg.load('waf_unit_test')
    cfg.load('mavgen')

    cfg.env.SUBMODULE_UPDATE = cfg.options.submodule_update

    cfg.start_msg('Source is git repository')
    if cfg.srcnode.find_node('.git'):
        cfg.end_msg('yes')
    else:
        cfg.end_msg('no')
        cfg.env.SUBMODULE_UPDATE = False

    cfg.msg('Update submodules', 'yes' if cfg.env.SUBMODULE_UPDATE else 'no')
    cfg.load('git_submodule')

    if cfg.options.enable_benchmarks:
        cfg.load('gbenchmark')
    cfg.load('gtest')
    cfg.load('static_linking')
    cfg.load('build_summary')

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

    # Always use system extensions
    cfg.define('_GNU_SOURCE', 1)

    cfg.write_config_header(os.path.join(cfg.variant, 'ap_config.h'))

    _collect_autoconfig_files(cfg)

def collect_dirs_to_recurse(bld, globs, **kw):
    dirs = []
    globs = Utils.to_list(globs)

    if bld.bldnode.is_child_of(bld.srcnode):
        kw['excl'] = Utils.to_list(kw.get('excl', []))
        kw['excl'].append(bld.bldnode.path_from(bld.srcnode))

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
        if not bld.env.HAS_GTEST:
            bld.fatal('check: gtest library is required')
        bld.options.clear_failed_tests = True

def _build_dynamic_sources(bld):
    bld(
        features='mavgen',
        source='modules/mavlink/message_definitions/v1.0/ardupilotmega.xml',
        output_dir='libraries/GCS_MAVLink/include/mavlink/v2.0/',
        name='mavlink',
        # this below is not ideal, mavgen tool should set this, but that's not
        # currently possible
        export_includes=[
            bld.bldnode.make_node('libraries').abspath(),
            bld.bldnode.make_node('libraries/GCS_MAVLink').abspath(),
        ],
    )

    def write_version_header(tsk):
        bld = tsk.generator.bld
        return bld.write_version_header(tsk.outputs[0].abspath())

    bld(
        name='ap_version',
        target='ap_version.h',
        vars=['AP_VERSION_ITEMS'],
        rule=write_version_header,
    )

    bld.env.prepend_value('INCLUDES', [
        bld.bldnode.abspath(),
    ])

def _build_common_taskgens(bld):
    # NOTE: Static library with vehicle set to UNKNOWN, shared by all
    # the tools and examples. This is the first step until the
    # dependency on the vehicles is reduced. Later we may consider
    # split into smaller pieces with well defined boundaries.
    bld.ap_stlib(
        name='ap',
        ap_vehicle='UNKNOWN',
        ap_libraries=bld.ap_get_all_libraries(),
    )

    if bld.env.HAS_GTEST:
        bld.libgtest(cxxflags=['-include', 'ap_config.h'])

    if bld.env.HAS_GBENCHMARK:
        bld.libbenchmark()

def _build_recursion(bld):
    common_dirs_patterns = [
        # TODO: Currently each vehicle also generate its own copy of the
        # libraries. Fix this, or at least reduce the amount of
        # vehicle-dependent libraries.
        '*',
        'Tools/*',
        'libraries/*/examples/*',
        'libraries/*/tests',
        'libraries/*/utility/tests',
        'libraries/*/benchmarks',
    ]

    common_dirs_excl = [
        'modules',
        'libraries/AP_HAL_*',
        'libraries/SITL',
    ]

    hal_dirs_patterns = [
        'libraries/%s/*/tests',
        'libraries/%s/*/benchmarks',
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

def _build_post_funs(bld):
    if bld.cmd == 'check':
        bld.add_post_fun(ardupilotwaf.test_summary)
    else:
        bld.build_summary_post_fun()

    if bld.env.SUBMODULE_UPDATE:
        bld.git_submodule_post_fun()

def build(bld):
    config_hash = Utils.h_file(bld.bldnode.make_node('ap_config.h').abspath())
    bld.env.CCDEPS = config_hash
    bld.env.CXXDEPS = config_hash

    bld.post_mode = Build.POST_LAZY

    bld.load('ardupilotwaf')

    bld.env.AP_LIBRARIES_OBJECTS_KW.update(
        use='mavlink',
        cxxflags=['-include', 'ap_config.h'],
    )

    _build_cmd_tweaks(bld)

    if bld.env.SUBMODULE_UPDATE:
        bld.add_group('git_submodules')
        for name in bld.env.GIT_SUBMODULES:
            bld.git_submodule(name)

    bld.add_group('dynamic_sources')
    _build_dynamic_sources(bld)

    bld.add_group('build')
    bld.get_board().build(bld)
    _build_common_taskgens(bld)

    _build_recursion(bld)

    _build_post_funs(bld)

ardupilotwaf.build_command('check',
    program_group_list='all',
    doc='builds all programs and run tests',
)
ardupilotwaf.build_command('check-all',
    program_group_list='all',
    doc='shortcut for `waf check --alltests`',
)

for name in ('antennatracker', 'copter', 'plane', 'rover'):
    ardupilotwaf.build_command(name,
        program_group_list=name,
        doc='builds %s programs' % name,
    )

for program_group in ('all', 'bin', 'tools', 'examples', 'tests', 'benchmarks'):
    ardupilotwaf.build_command(program_group,
        program_group_list=program_group,
        doc='builds all programs of %s group' % program_group,
    )

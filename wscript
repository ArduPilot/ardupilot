#!/usr/bin/env python
# encoding: utf-8

from __future__ import print_function

import os.path
import os
import sys
import subprocess
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

# Default installation prefix for Linux boards
default_prefix = '/usr/'

# Override Build execute and Configure post_recurse methods for autoconfigure purposes
Build.BuildContext.execute = ardupilotwaf.ap_autoconfigure(Build.BuildContext.execute)
Configure.ConfigurationContext.post_recurse = ardupilotwaf.ap_configure_post_recurse()


def _set_build_context_variant(board):
    for c in Context.classes:
        if not issubclass(c, Build.BuildContext):
            continue
        c.variant = board

def init(ctx):
    env = ConfigSet.ConfigSet()
    try:
        p = os.path.join(Context.out_dir, Build.CACHE_DIR, Build.CACHE_SUFFIX)
        env.load(p)
    except EnvironmentError:
        return

    Configure.autoconfig = 'clobber' if env.AUTOCONFIG else False

    board = ctx.options.board or env.BOARD

    if not board:
        return

    # define the variant build commands according to the board
    _set_build_context_variant(board)

def options(opt):
    opt.load('compiler_cxx compiler_c waf_unit_test python')
    opt.load('ardupilotwaf')
    opt.load('build_summary')

    g = opt.ap_groups['configure']

    boards_names = boards.get_boards_names()
    removed_names = boards.get_removed_boards()
    g.add_option('--board',
        action='store',
        default=None,
        help='Target board to build, choices are %s.' % ', '.join(boards_names))

    g.add_option('--debug',
        action='store_true',
        default=False,
        help='Configure as debug variant.')

    g.add_option('--toolchain',
        action='store',
        default=None,
        help='Override default toolchain used for the board. Use "native" for using the host toolchain.')

    g.add_option('--disable-gccdeps',
        action='store_true',
        default=False,
        help='Disable the use of GCC dependencies output method and use waf default method.')

    g.add_option('--enable-asserts',
        action='store_true',
        default=False,
        help='enable OS level asserts.')

    g.add_option('--use-nuttx-iofw',
        action='store_true',
        default=False,
        help='use old NuttX IO firmware for IOMCU')

    g.add_option('--bootloader',
        action='store_true',
        default=False,
        help='Configure for building a bootloader.')

    g.add_option('--no-autoconfig',
        dest='autoconfig',
        action='store_false',
        default=True,
        help='''Disable autoconfiguration feature. By default, the build system
triggers a reconfiguration whenever it thinks it's necessary - this
option disables that.
''')

    g.add_option('--no-submodule-update',
        dest='submodule_update',
        action='store_false',
        default=True,
        help='''Don't update git submodules. Useful for building with
submodules at specific revisions.
''')

    g.add_option('--enable-header-checks', action='store_true',
        default=False,
        help="Enable checking of headers")

    g.add_option('--default-parameters',
        default=None,
        help='set default parameters to embed in the firmware')

    g.add_option('--enable-math-check-indexes',
                 action='store_true',
                 default=False,
                 help="Enable checking of math indexes")

    g.add_option('--enable-scripting', action='store_true',
                 default=False,
                 help="Enable onboard scripting engine")

    g.add_option('--scripting-checks', action='store_true',
                 default=True,
                 help="Enable runtime scripting sanity checks")

    g = opt.ap_groups['linux']

    linux_options = ('--prefix', '--destdir', '--bindir', '--libdir')
    for k in linux_options:
        option = opt.parser.get_option(k)
        if option:
            opt.parser.remove_option(k)
            g.add_option(option)

    g.add_option('--apstatedir',
        action='store',
        default='',
        help='''Where to save data like parameters, log and terrain.
This is the --localstatedir + ArduPilots subdirectory [default:
board-dependent, usually /var/lib/ardupilot]''')

    g.add_option('--rsync-dest',
        dest='rsync_dest',
        action='store',
        default='',
        help='''Destination for the rsync Waf command. It can be passed during
configuration in order to save typing.
''')

    g.add_option('--enable-benchmarks',
        action='store_true',
        default=False,
        help='Enable benchmarks.')

    g.add_option('--enable-lttng', action='store_true',
        default=False,
        help="Enable lttng integration")

    g.add_option('--disable-libiio', action='store_true',
        default=False,
        help="Don't use libiio even if supported by board and dependencies available")

    g.add_option('--disable-tests', action='store_true',
        default=False,
        help="Disable compilation and test execution")

    g.add_option('--enable-sfml', action='store_true',
                 default=False,
                 help="Enable SFML graphics library")

    g.add_option('--enable-sfml-audio', action='store_true',
                 default=False,
                 help="Enable SFML audio library")

    g.add_option('--sitl-osd', action='store_true',
                 default=False,
                 help="Enable SITL OSD")

    g.add_option('--sitl-rgbled', action='store_true',
                 default=False,
                 help="Enable SITL RGBLed")

    g.add_option('--sitl-flash-storage',
        action='store_true',
        default=False,
        help='Configure for building SITL with flash storage emulation.')
    
    g.add_option('--static',
        action='store_true',
        default=False,
        help='Force a static build')

    g.add_option("--enable-gcov",
        help=("Enable gcov code coverage analysis."
             " WARNING: this option only has effect "
             "with the configure command. "
             "You should also add --lcov-report to your build command."),
        action="store_true", default=False,
        dest="enable_gcov")

    g.add_option("--lcov-report",
        help=("Generates a lcov code coverage report "
             "(use this option at build time, not in configure)"),
        action="store_true", default=False,
        dest="lcov_report")



# EXECUTE enough code via the autotest tool to see coverage results afterwards, but don't build/rebuild anything
# our aim here is to try to execute as many code path/s as we have available to us, and we'll afterward report
# on the percentage of code executed and not executed etc.
def	run_coverage_tests(bld):

    FNULL = open(os.devnull, 'w')

    #tests = ['fly.ArduPlane']
    #tests = ['fly.ArduCopter','fly.ArduPlane']
    tests = ['fly.ArduCopter','fly.ArduPlane', 'fly.QuadPlane', 'drive.APMrover2', 'dive.ArduSub']

    for test in tests:
        print("LCOV/GCOV -> "+test+" started.... this will take quite some time...")
        testcmd = '( ./Tools/autotest/autotest.py --speedup=5 --timeout=14400 --debug --no-configure '+test+' ) '
        print("Coverage Tests Executing:"+testcmd+" > ./GCOV_"+test+".log")
        FLOG = open("./GCOV_"+test+".log", 'w')
        if subprocess.Popen(testcmd, shell=True , stdout=FLOG, stderr=FNULL).wait():
	        print("LCOV/GCOV -> "+test+" see ./GCOV_"+test+".log for log of activity)")
	        raise SystemExit(1)
        print("LCOV/GCOV -> "+test+" succeeded")
        FLOG.close()

    #TODO add any other execution path/s we can to maximise the actually used code, can we run other tests or things?
    # eg run.unit_tests, run.examples , test.AntennaTracker or other things?


def lcov_report(bld):
    """
    Generates the coverage report
    :param bld: temporal options context
    :type bld: wscript.tmp
    """
    env = bld.env
    REPORTS = "reports"

    if not env.GCOV_ENABLED:
        raise WafError("project not configured for code coverage;"
                       " reconfigure with --enable-gcov")

    run_coverage_tests(bld)
    lcov_report_dir = os.path.join(REPORTS, "lcov-report")
    try:
        if not os.path.exists(REPORTS):
            os.mkdir(REPORTS)

        create_dir_command = "rm -rf " + lcov_report_dir
        create_dir_command += " && mkdir " + lcov_report_dir

        print (create_dir_command );
        if subprocess.Popen(create_dir_command, shell=True).wait():
            raise SystemExit(1)

        info_file = os.path.join(lcov_report_dir, "lcov.info")

        FLOG = open("GCOV_lcov.log", 'w')
        FNULL = open(os.devnull, 'w')

        lcov_command =\
            "lcov --no-external --capture --directory . -o " + info_file
        lcov_command +=\
            " && lcov --remove " + info_file + " \".waf*\" -o " + info_file

        print("LCOV/GCOV executing lcov -> ")
        print ("\t"+lcov_command + " > ./GCOV_lcov.log")
        if subprocess.Popen(lcov_command, shell=True, stdout=FLOG, stderr=FNULL).wait():
            raise SystemExit(1)
        FLOG.close()


        FLOG = open("GCOV_genhtml.log", 'w')
        genhtml_command = "genhtml " + info_file
        genhtml_command += " -o " + lcov_report_dir
        print("LCOV/GCOV building html report -> ")
        print ("\t"+genhtml_command + " > ./GCOV_genhtml.log")
        if subprocess.Popen(genhtml_command, shell=True, stdout=FLOG, stderr=FNULL).wait():
            raise SystemExit(1)
        FLOG.close()

    except:
        print (\
            "LCOV/GCOV -> Problems running coverage. Try manually" );

    finally:
        print (\
            "LCOV/GCOV -> Coverage successful. Open " + lcov_report_dir +\
            "/index.html" );


def _collect_autoconfig_files(cfg):
    for m in sys.modules.values():
        paths = []
        if hasattr(m, '__file__') and m.__file__ is not None:
            paths.append(m.__file__)
        elif hasattr(m, '__path__'):
            for p in m.__path__:
                if p is not None:
                    paths.append(p)

        for p in paths:
            if p in cfg.files or not os.path.isfile(p):
                continue

            with open(p, 'rb') as f:
                cfg.hash = Utils.h_list((cfg.hash, f.read()))
                cfg.files.append(p)

def configure(cfg):
	# we need to enable debug mode when building for gconv, and force it to sitl
    if cfg.options.enable_gcov:
        cfg.options.debug = True
        cfg.options.board = 'sitl'

    if cfg.options.board is None:
        cfg.options.board = 'sitl'

    boards_names = boards.get_boards_names()
    if not cfg.options.board in boards_names:
        for b in boards_names:
            if b.upper() == cfg.options.board.upper():
                cfg.options.board = b
                break
        
    cfg.env.BOARD = cfg.options.board
    cfg.env.DEBUG = cfg.options.debug
    cfg.env.AUTOCONFIG = cfg.options.autoconfig

    _set_build_context_variant(cfg.env.BOARD)
    cfg.setenv(cfg.env.BOARD)

    cfg.env.BOARD = cfg.options.board
    cfg.env.DEBUG = cfg.options.debug
    cfg.env.ENABLE_ASSERTS = cfg.options.enable_asserts
    cfg.env.BOOTLOADER = cfg.options.bootloader
    cfg.env.USE_NUTTX_IOFW = cfg.options.use_nuttx_iofw

    cfg.env.OPTIONS = cfg.options.__dict__

    # Allow to differentiate our build from the make build
    cfg.define('WAF_BUILD', 1)

    cfg.msg('Autoconfiguration', 'enabled' if cfg.options.autoconfig else 'disabled')

    #Sets the lcov flag if is configurated
    if cfg.options.enable_gcov:
        cfg.start_msg("GCOV code coverage analysis")
        cfg.env.GCOV_ENABLED = True
        cfg.env.append_value('CCFLAGS', '-fprofile-arcs')
        cfg.env.append_value('CCFLAGS', '-ftest-coverage')
        cfg.env.append_value('CXXFLAGS', '-fprofile-arcs')
        cfg.env.append_value('CXXFLAGS', '-ftest-coverage')
        cfg.env.append_value('LINKFLAGS', '-lgcov')
        cfg.env.append_value('LINKFLAGS', '-coverage')
        cfg.end_msg('yes' , color='RED')
    else:
        cfg.start_msg("GCOV code coverage analysis")
        cfg.end_msg('no' , color='GREEN')

    if cfg.options.static:
        cfg.msg('Using static linking', 'yes', color='YELLOW')
        cfg.env.STATIC_LINKING = True

    cfg.load('ap_library')

    cfg.msg('Setting board to', cfg.options.board)
    cfg.get_board().configure(cfg)

    cfg.load('clang_compilation_database')
    cfg.load('waf_unit_test')
    cfg.load('mavgen')
    cfg.load('uavcangen')

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

    cfg.start_msg('Scripting')
    if cfg.options.enable_scripting:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Scripting runtime checks')
    if cfg.options.scripting_checks:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.env.append_value('GIT_SUBMODULES', 'mavlink')

    cfg.env.prepend_value('INCLUDES', [
        cfg.srcnode.abspath() + '/libraries/',
    ])

    cfg.find_program('rsync', mandatory=False)
    if cfg.options.rsync_dest:
        cfg.msg('Setting rsync destination to', cfg.options.rsync_dest)
        cfg.env.RSYNC_DEST = cfg.options.rsync_dest

    if cfg.options.enable_header_checks:
        cfg.msg('Enabling header checks', cfg.options.enable_header_checks)
        cfg.env.ENABLE_HEADER_CHECKS = True
    else:
        cfg.env.ENABLE_HEADER_CHECKS = False

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

def board(ctx):
    env = ConfigSet.ConfigSet()
    try:
        p = os.path.join(Context.out_dir, Build.CACHE_DIR, Build.CACHE_SUFFIX)
        env.load(p)
    except:
        print('No board currently configured')
        return

    print('Board configured to: {}'.format(env.BOARD))

def _build_cmd_tweaks(bld):
    if bld.cmd == 'check-all':
        bld.options.all_tests = True
        bld.cmd = 'check'

    if bld.cmd == 'check':
        if not bld.env.HAS_GTEST:
            bld.fatal('check: gtest library is required')
        bld.options.clear_failed_tests = True

def _build_dynamic_sources(bld):
    if not bld.env.BOOTLOADER:
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

    if bld.get_board().with_uavcan or bld.env.HAL_WITH_UAVCAN==True:
        bld(
            features='uavcangen',
            source=bld.srcnode.ant_glob('modules/uavcan/dsdl/* libraries/AP_UAVCAN/dsdl/*', dir=True, src=False),
            output_dir='modules/uavcan/libuavcan/include/dsdlc_generated',
            name='uavcan',
            export_includes=[
                bld.bldnode.make_node('modules/uavcan/libuavcan/include/dsdlc_generated').abspath(),
            ]
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
        'libraries/%s/tests',
        'libraries/%s/*/tests',
        'libraries/%s/*/benchmarks',
        'libraries/%s/examples/*',
    ]

    dirs_to_recurse = collect_dirs_to_recurse(
        bld,
        common_dirs_patterns,
        excl=common_dirs_excl,
    )
    if bld.env.IOMCU_FW is not None:
        if bld.env.IOMCU_FW:
            dirs_to_recurse.append('libraries/AP_IOMCU/iofirmware')
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

    if bld.env.GCOV_ENABLED:
      bld.add_post_fun(lcov_report)

def _load_pre_build(bld):
    '''allow for a pre_build() function in build modules'''
    brd = bld.get_board()
    if getattr(brd, 'pre_build', None):
        brd.pre_build(bld)    

def build(bld):
    config_hash = Utils.h_file(bld.bldnode.make_node('ap_config.h').abspath())
    bld.env.CCDEPS = config_hash
    bld.env.CXXDEPS = config_hash

    bld.post_mode = Build.POST_LAZY

    bld.load('ardupilotwaf')

    bld.env.AP_LIBRARIES_OBJECTS_KW.update(
        use=['mavlink'],
        cxxflags=['-include', 'ap_config.h'],
    )

    _load_pre_build(bld)

    if bld.get_board().with_uavcan:
        bld.env.AP_LIBRARIES_OBJECTS_KW['use'] += ['uavcan']

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

for name in ('antennatracker', 'copter', 'heli', 'plane', 'rover', 'sub', 'bootloader','iofirmware'):
    ardupilotwaf.build_command(name,
        program_group_list=name,
        doc='builds %s programs' % name,
    )

for program_group in ('all', 'bin', 'tools', 'examples', 'tests', 'benchmarks'):
    ardupilotwaf.build_command(program_group,
        program_group_list=program_group,
        doc='builds all programs of %s group' % program_group,
    )

class LocalInstallContext(Build.InstallContext):
    """runs install using BLD/install as destdir, where BLD is the build variant directory"""
    cmd = 'localinstall'

    def __init__(self, **kw):
        super(LocalInstallContext, self).__init__(**kw)
        self.local_destdir = os.path.join(self.variant_dir, 'install')

    def execute(self):
        old_destdir = self.options.destdir
        self.options.destdir = self.local_destdir
        r = super(LocalInstallContext, self).execute()
        self.options.destdir = old_destdir
        return r

class RsyncContext(LocalInstallContext):
    """runs localinstall and then rsyncs BLD/install with the target system"""
    cmd = 'rsync'

    def __init__(self, **kw):
        super(RsyncContext, self).__init__(**kw)
        self.add_pre_fun(RsyncContext.create_rsync_taskgen)

    def create_rsync_taskgen(self):
        if 'RSYNC' not in self.env:
            self.fatal('rsync program seems not to be installed, can\'t continue')

        self.add_group()

        tg = self(
            name='rsync',
            rule='${RSYNC} -a ${RSYNC_SRC}/ ${RSYNC_DEST}',
            always=True,
        )

        tg.env.RSYNC_SRC = self.local_destdir
        if self.options.rsync_dest:
            self.env.RSYNC_DEST = self.options.rsync_dest

        if 'RSYNC_DEST' not in tg.env:
            self.fatal('Destination for rsync not defined. Either pass --rsync-dest here or during configuration.')

        tg.post()

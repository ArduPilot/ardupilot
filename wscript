#!/usr/bin/env python3
# encoding: utf-8
# flake8: noqa

import optparse
import os.path
import os
import sys
import subprocess
import json
import fnmatch
sys.path.insert(0, 'Tools/ardupilotwaf/')
sys.path.insert(0, 'Tools/scripts/')

import ardupilotwaf
import boards
import shutil
import build_options
import glob

from waflib import Build, ConfigSet, Configure, Context, Utils
from waflib.Configure import conf

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


# Get the GitHub Actions summary file path
is_ci = os.getenv('CI')


def _set_build_context_variant(board):
    for c in Context.classes:
        if not issubclass(c, Build.BuildContext):
            continue
        c.variant = board

# Remove all submodules and then sync
@conf
def submodule_force_clean(ctx):
    whitelist = {
                            'COLCON_IGNORE',
                            'esp_idf',
                          }

    # Get all items in the modules folder
    module_list = os.scandir('modules')

    # Delete all directories except those in the whitelist
    for module in module_list:
        if (module.is_dir()) and (module.name not in whitelist):
            shutil.rmtree(module)

    submodulesync(ctx)

# run Tools/gittools/submodule-sync.sh to sync submodules
@conf
def submodulesync(ctx):
    subprocess.call(['Tools/gittools/submodule-sync.sh'])

def init(ctx):
    # Generate Task List, so that VS Code extension can keep track
    # of changes to possible build targets
    generate_tasklist(ctx, False)
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

def add_build_options(g):
    '''add any option in Tools/scripts/build_options.py'''
    for opt in build_options.BUILD_OPTIONS:
        enable_option = "--" + opt.config_option()
        disable_option = enable_option.replace("--enable", "--disable")
        enable_description = opt.description
        if not enable_description.lower().startswith("enable"):
            enable_description = "Enable " + enable_description
        disable_description = "Disable " + enable_description[len("Enable "):]
        g.add_option(enable_option,
                     action='store_true',
                     default=False,
                     help=enable_description)
        g.add_option(disable_option,
                     action='store_true',
                     default=False,
                     help=disable_description)

        # also add entirely-lower-case equivalents with underscores
        # replaced with dashes::
        lower_enable_option = enable_option.lower().replace("_", "-")
        if lower_enable_option != enable_option:
            g.add_option(lower_enable_option,
                         action='store_true',
                         default=False,
                         help=optparse.SUPPRESS_HELP)
        lower_disable_option = disable_option.lower().replace("_", "-")
        if lower_disable_option != disable_option:
            g.add_option(lower_disable_option,
                         action='store_true',
                         default=False,
                         help=optparse.SUPPRESS_HELP)

def add_script_options(g):
    '''add any drivers or applets from libraries/AP_Scripting'''
    driver_list = glob.glob(os.path.join(Context.run_dir, "libraries/AP_Scripting/drivers/*.lua"))
    applet_list = glob.glob(os.path.join(Context.run_dir, "libraries/AP_Scripting/applets/*.lua"))
    for d in driver_list + applet_list:
        bname = os.path.basename(d)
        embed_name = bname[:-4]
        embed_option = "--embed-%s" % embed_name
        g.add_option(embed_option,
                     action='store_true',
                     default=False,
                     help="Embed %s in ROMFS" % bname)

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

    g.add_option('--debug-symbols', '-g',
        action='store_true',
        default=False,
        help='Add debug symbolds to build.')

    g.add_option('--vs-launch',
        action='store_true',
        default=False,
        help='Generate wscript environment variable to .vscode/setting.json for Visual Studio Code')
    
    g.add_option('--disable-watchdog',
        action='store_true',
        default=False,
        help='Build with watchdog disabled.')

    g.add_option('--coverage',
                 action='store_true',
                 default=False,
                 help='Configure coverage flags.')

    g.add_option('--Werror',
        action='store_true',
        default=None,
        help='build with -Werror.')

    g.add_option('--disable-Werror',
        action='store_true',
        default=None,
        help='Disable -Werror.')
    
    g.add_option('--toolchain',
        action='store',
        default=None,
        help='Override default toolchain used for the board. Use "native" for using the host toolchain.')

    g.add_option('--enable-asserts',
        action='store_true',
        default=False,
        help='enable OS level asserts.')

    g.add_option('--save-temps',
        action='store_true',
        default=False,
        help='save compiler temporary files.')
    
    g.add_option('--enable-malloc-guard',
        action='store_true',
        default=False,
        help='enable malloc guard regions.')

    g.add_option('--enable-stats',
        action='store_true',
        default=False,
        help='enable OS level thread statistics.')

    g.add_option('--bootloader',
        action='store_true',
        default=False,
        help='Configure for building a bootloader.')

    g.add_option('--signed-fw',
        action='store_true',
        default=False,
        help='Configure for signed firmware support.')

    g.add_option('--private-key',
                 action='store',
                 default=None,
            help='path to private key for signing firmware.')
    
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

    g.add_option('--disable-scripting', action='store_true',
                 default=False,
                 help="Disable onboard scripting engine")

    g.add_option('--enable-scripting', action='store_true',
                 default=False,
                 help="Enable onboard scripting engine")

    g.add_option('--no-gcs', action='store_true',
                 default=False,
                 help="Disable GCS code")
    
    g.add_option('--scripting-checks', action='store_true',
                 default=True,
                 help="Enable runtime scripting sanity checks")

    g.add_option('--enable-onvif', action='store_true',
                 default=False,
                 help="Enables and sets up ONVIF camera control")

    g.add_option('--scripting-docs', action='store_true',
                 default=False,
                 help="enable generation of scripting documentation")

    g.add_option('--enable-opendroneid', action='store_true',
                 default=False,
                 help="Enables OpenDroneID")

    g.add_option('--enable-check-firmware', action='store_true',
                 default=False,
                 help="Enables firmware ID checking on boot")

    g.add_option('--enable-custom-controller', action='store_true',
                 default=False,
                 help="Enables custom controller")

    g.add_option('--enable-gps-logging', action='store_true',
                 default=False,
                 help="Enables GPS logging")

    g.add_option('--disable-networking', action='store_true',
                 help="Disable the networking API code")

    g.add_option('--enable-networking-tests', action='store_true',
                 help="Enable the networking test code. Automatically enables networking.")
    
    g.add_option('--enable-dronecan-tests', action='store_true',
                 default=False,
                 help="Enables DroneCAN tests in sitl")

    g.add_option('--sitl-littlefs', action='store_true',
                 default=False,
                 help="Enable littlefs for filesystem access on SITL (under construction)")

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

    g.add_option('--enable-sfml-joystick', action='store_true',
                 default=False,
                 help="Enable SFML joystick input library")

    g.add_option('--enable-sfml-audio', action='store_true',
                 default=False,
                 help="Enable SFML audio library")

    g.add_option('--osd', action='store_true',
                 default=False,
                 help="Enable OSD support")

    g.add_option('--osd-fonts', action='store_true',
                 default=False,
                 help="Enable OSD support with fonts")
    
    g.add_option('--sitl-osd', action='store_true',
                 default=False,
                 help="Enable SITL OSD")

    g.add_option('--sitl-rgbled', action='store_true',
                 default=False,
                 help="Enable SITL RGBLed")

    g.add_option('--force-32bit', action='store_true',
                 default=False,
                 help="Force 32bit build")

    g.add_option('--build-dates', action='store_true',
                 default=False,
                 help="Include build date in binaries.  Appears in AUTOPILOT_VERSION.os_sw_version")

    g.add_option('--sitl-flash-storage',
        action='store_true',
        default=False,
        help='Use flash storage emulation.')

    g.add_option('--ekf-double',
        action='store_true',
        default=False,
        help='Configure EKF as double precision.')

    g.add_option('--ekf-single',
        action='store_true',
        default=False,
        help='Configure EKF as single precision.')
    
    g.add_option('--static',
        action='store_true',
        default=False,
        help='Force a static build')

    g.add_option('--postype-single',
        action='store_true',
        default=False,
        help='force single precision postype_t')

    g.add_option('--consistent-builds',
        action='store_true',
        default=False,
        help='force consistent build outputs for things like __LINE__ and build hashes')

    g.add_option('--extra-hwdef',
	    action='store',
	    default=None,
	    help='Extra hwdef.dat file for custom build.')

    g.add_option('--assert-cc-version',
                 default=None,
                 help='fail configure if not using the specified gcc version')

    g.add_option('--num-aux-imus',
                 type='int',
                 default=0,
                 help='number of auxiliary IMUs')

    g.add_option('--board-start-time',
                 type='int',
                 default=0,
                 help='zero time on boot in microseconds')

    g.add_option('--enable-iomcu-profiled-support',
                    action='store_true',
                    default=False,
                    help='enable iomcu profiled support')

    g.add_option('--enable-new-checking',
        action='store_true',
        default=False,
        help='enables checking of new to ensure NEW_NOTHROW is used')

    # support enabling any option in build_options.py
    add_build_options(g)

    # support embedding lua drivers and applets
    add_script_options(g)
    
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
    if is_ci:
        print(f"::group::Waf Configure")
	# we need to enable debug mode when building for gconv, and force it to sitl
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
    cfg.env.DEBUG_SYMBOLS = cfg.options.debug_symbols
    cfg.env.COVERAGE = cfg.options.coverage
    cfg.env.AUTOCONFIG = cfg.options.autoconfig

    _set_build_context_variant(cfg.env.BOARD)
    cfg.setenv(cfg.env.BOARD)

    if cfg.options.signed_fw:
        cfg.env.AP_SIGNED_FIRMWARE = True
        cfg.options.enable_check_firmware = True

    cfg.env.BOARD = cfg.options.board
    cfg.env.DEBUG = cfg.options.debug
    cfg.env.VS_LAUNCH = cfg.options.vs_launch
    cfg.env.DEBUG_SYMBOLS = cfg.options.debug_symbols
    cfg.env.COVERAGE = cfg.options.coverage
    cfg.env.FORCE32BIT = cfg.options.force_32bit
    cfg.env.ENABLE_ASSERTS = cfg.options.enable_asserts
    cfg.env.BOOTLOADER = cfg.options.bootloader
    cfg.env.ENABLE_MALLOC_GUARD = cfg.options.enable_malloc_guard
    cfg.env.ENABLE_STATS = cfg.options.enable_stats
    cfg.env.SAVE_TEMPS = cfg.options.save_temps
    cfg.env.CONSISTENT_BUILDS = cfg.options.consistent_builds

    extra_hwdef = cfg.options.extra_hwdef
    if extra_hwdef is not None and not os.path.exists(extra_hwdef):
        raise FileNotFoundError(f"extra-hwdef file NOT found: '{cfg.options.extra_hwdef}'")
    cfg.env.HWDEF_EXTRA = cfg.options.extra_hwdef
    if cfg.env.HWDEF_EXTRA:
        cfg.env.HWDEF_EXTRA = os.path.abspath(cfg.env.HWDEF_EXTRA)

    cfg.env.OPTIONS = cfg.options.__dict__

    # Allow to differentiate our build from the make build
    cfg.define('WAF_BUILD', 1)

    cfg.msg('Autoconfiguration', 'enabled' if cfg.options.autoconfig else 'disabled')

    if cfg.options.static:
        cfg.msg('Using static linking', 'yes', color='YELLOW')
        cfg.env.STATIC_LINKING = True

    if cfg.options.num_aux_imus > 0:
        cfg.define('INS_AUX_INSTANCES', cfg.options.num_aux_imus)

    if cfg.options.board_start_time != 0:
        cfg.define('AP_BOARD_START_TIME', cfg.options.board_start_time)
        # also in env for hrt.c
        cfg.env.AP_BOARD_START_TIME = cfg.options.board_start_time

    # require python 3.8.x or later
    # also update `MIN_VER` in `./waf`
    cfg.load('python')
    cfg.check_python_version(minver=(3,8,0))

    cfg.load('ap_library')

    cfg.msg('Setting board to', cfg.options.board)
    cfg.get_board().configure(cfg)

    cfg.load('waf_unit_test')
    cfg.load('mavgen')
    cfg.load('dronecangen')

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

    if cfg.env.BOARD == "sitl":
        cfg.start_msg('Littlefs')

        if cfg.options.sitl_littlefs:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

    cfg.load('littlefs')
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
    if cfg.options.disable_scripting:
        cfg.end_msg('disabled', color='YELLOW')
    elif cfg.options.enable_scripting:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('maybe')
    cfg.recurse('libraries/AP_Scripting')

    cfg.recurse('libraries/AP_GPS')
    cfg.recurse('libraries/AP_HAL_SITL')
    cfg.recurse('libraries/SITL')

    cfg.recurse('libraries/AP_Networking')
    cfg.recurse('libraries/AP_DDS')

    cfg.start_msg('Scripting runtime checks')
    if cfg.options.scripting_checks:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Debug build')
    if cfg.env.DEBUG:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    if cfg.env.DEBUG:
        cfg.start_msg('VS Code launch')
        if cfg.env.VS_LAUNCH:
            cfg.end_msg('enabled')
        else:
            cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Coverage build')
    if cfg.env.COVERAGE:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Consistent build')
    if cfg.env.CONSISTENT_BUILDS:
        cfg.end_msg('enabled')
    else:
        cfg.end_msg('disabled', color='YELLOW')

    cfg.start_msg('Force 32-bit build')
    if cfg.env.FORCE32BIT:
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

    # Always use system extensions
    cfg.define('_GNU_SOURCE', 1)

    if cfg.options.Werror:
        # print(cfg.options.Werror)
        if cfg.options.disable_Werror:
            cfg.options.Werror = False

    cfg.write_config_header(os.path.join(cfg.variant, 'ap_config.h'), guard='_AP_CONFIG_H_')

    # add in generated flags
    cfg.env.CXXFLAGS += ['-include', 'ap_config.h']

    cfg.remove_target_list()
    _collect_autoconfig_files(cfg)
    if is_ci:
        print("::endgroup::")

    if cfg.env.DEBUG and cfg.env.VS_LAUNCH:
        import vscode_helper
        vscode_helper.init_launch_json_if_not_exist(cfg)
        vscode_helper.update_openocd_cfg(cfg)

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

def list_ap_periph_boards(ctx):
    print(*boards.get_ap_periph_boards())

@conf
def ap_periph_boards(ctx):
    return boards.get_ap_periph_boards()

vehicles = ['antennatracker', 'blimp', 'copter', 'heli', 'plane', 'rover', 'sub']

def generate_tasklist(ctx, do_print=True):
    boardlist = boards.get_boards_names()
    ap_periph_targets = boards.get_ap_periph_boards()
    tasks = []
    with open(os.path.join(Context.top_dir, "tasklist.json"), "w") as tlist:
        for board in boardlist:
            task = {}
            task['configure'] = board
            if board in ap_periph_targets:
                if 'sitl' not in board:
                    # we only support AP_Periph and bootloader builds
                    task['targets'] = ['AP_Periph', 'bootloader']
                else:
                    task['targets'] = ['AP_Periph']
            elif 'iofirmware' in board:
                task['targets'] = ['iofirmware', 'bootloader']
            else:
                if boards.is_board_based(board, boards.sitl):
                    task['targets'] = vehicles + ['replay']
                elif boards.is_board_based(board, boards.LinuxBoard):
                    task['targets'] = vehicles
                else:
                    task['targets'] = vehicles + ['bootloader']
                    task['buildOptions'] = '--upload'
            tasks.append(task)
        tlist.write(json.dumps(tasks))
        if do_print:
            print(json.dumps(tasks))

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
            source='modules/mavlink/message_definitions/v1.0/all.xml',
            output_dir='libraries/GCS_MAVLink/include/mavlink/v2.0/',
            name='mavlink',
            # this below is not ideal, mavgen tool should set this, but that's not
            # currently possible
            export_includes=[
            bld.bldnode.make_node('libraries').abspath(),
            bld.bldnode.make_node('libraries/GCS_MAVLink').abspath(),
            ],
            )

    if (bld.get_board().with_can or bld.env.HAL_NUM_CAN_IFACES) and not bld.env.AP_PERIPH:
        bld(
            features='dronecangen',
            source=bld.srcnode.ant_glob('modules/DroneCAN/DSDL/[a-z]* libraries/AP_DroneCAN/dsdl/[a-z]*', dir=True, src=False),
            output_dir='modules/DroneCAN/libcanard/dsdlc_generated/',
            name='dronecan',
            export_includes=[
                bld.bldnode.make_node('modules/DroneCAN/libcanard/dsdlc_generated/include').abspath(),
                bld.srcnode.find_dir('modules/DroneCAN/libcanard/').abspath(),
                bld.srcnode.find_dir('libraries/AP_DroneCAN/canard/').abspath(),
                ]
            )
    elif bld.env.AP_PERIPH:
        bld(
            features='dronecangen',
            source=bld.srcnode.ant_glob('modules/DroneCAN/DSDL/* libraries/AP_DroneCAN/dsdl/*', dir=True, src=False),
            output_dir='modules/DroneCAN/libcanard/dsdlc_generated/',
            name='dronecan',
            export_includes=[
                bld.bldnode.make_node('modules/DroneCAN/libcanard/dsdlc_generated/include').abspath(),
                bld.srcnode.find_dir('modules/DroneCAN/libcanard/').abspath(),
            ]
        )

    bld.recurse("libraries/AP_DDS")

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

    if bld.env.PERIPH_FW is not None:
        if bld.env.PERIPH_FW:
            dirs_to_recurse.append('Tools/AP_Periph')

    dirs_to_recurse.append('libraries/AP_Scripting')

    if bld.env.ENABLE_ONVIF:
        dirs_to_recurse.append('libraries/AP_ONVIF')

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

def _load_pre_build(bld):
    '''allow for a pre_build() function in build modules'''
    if bld.cmd == 'clean':
        return
    brd = bld.get_board()
    if getattr(brd, 'pre_build', None):
        brd.pre_build(bld)    

def build(bld):
    if is_ci:
        print(f"::group::Waf Build")
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

    if bld.get_board().with_can:
        bld.env.AP_LIBRARIES_OBJECTS_KW['use'] += ['dronecan']

    if bld.get_board().with_littlefs:
        bld.env.AP_LIBRARIES_OBJECTS_KW['use'] += ['littlefs']
        bld.littlefs()

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
    if is_ci:
        def print_ci_endgroup(bld):
            print(f"::endgroup::")
        bld.add_post_fun(print_ci_endgroup)


    if bld.env.DEBUG and bld.env.VS_LAUNCH:
        import vscode_helper
        vscode_helper.update_settings(bld)

ardupilotwaf.build_command('check',
    program_group_list='all',
    doc='builds all programs and run tests',
)
ardupilotwaf.build_command('check-all',
    program_group_list='all',
    doc='shortcut for `waf check --alltests`',
)

for name in (vehicles + ['bootloader','iofirmware','AP_Periph','replay']):
    ardupilotwaf.build_command(name,
        program_group_list=name,
        doc='builds %s programs' % name,
    )

for program_group in ('all', 'bin', 'tool', 'examples', 'tests', 'benchmarks'):
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

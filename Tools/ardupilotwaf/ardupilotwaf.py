# encoding: utf-8

# flake8: noqa

from waflib import Build, ConfigSet, Configure, Context, Errors, Logs, Options, Utils, Task
from waflib.Configure import conf
from waflib.Scripting import run_command
from waflib.TaskGen import before_method, after_method, feature
import os.path, os
from pathlib import Path
from collections import OrderedDict
import subprocess

SOURCE_EXTS = [
    '*.S',
    '*.c',
    '*.cpp',
]

COMMON_VEHICLE_DEPENDENT_CAN_LIBRARIES = [
    'AP_CANManager',
    'AP_KDECAN',
    'AP_PiccoloCAN',
    'AP_PiccoloCAN/piccolo_protocol',
]

COMMON_VEHICLE_DEPENDENT_LIBRARIES = [
    'AP_AccelCal',
    'AP_ADC',
    'AP_AHRS',
    'AP_Airspeed',
    'AP_Baro',
    'AP_BattMonitor',
    'AP_BoardConfig',
    'AP_Camera',
    'AP_Common',
    'AP_Compass',
    'AP_Declination',
    'AP_GPS',
    'AP_GSOF',
    'AP_HAL',
    'AP_HAL_Empty',
    'AP_DDS',
    'AP_InertialSensor',
    'AP_Math',
    'AP_Mission',
    'AP_DAL',
    'AP_NavEKF',
    'AP_NavEKF2',
    'AP_NavEKF3',
    'AP_Notify',
    'AP_OpticalFlow',
    'AP_Param',
    'AP_Rally',
    'AP_LightWareSerial',
    'AP_RangeFinder',
    'AP_Scheduler',
    'AP_SerialManager',
    'AP_Terrain',
    'AP_Vehicle',
    'AP_InternalError',
    'AP_Logger',
    'Filter',
    'GCS_MAVLink',
    'RC_Channel',
    'SRV_Channel',
    'StorageManager',
    'AP_Tuning',
    'AP_RPM',
    'AP_RSSI',
    'AP_Mount',
    'AP_Module',
    'AP_Button',
    'AP_ICEngine',
    'AP_Networking',
    'AP_Frsky_Telem',
    'AP_IBus_Telem',
    'AP_FlashStorage',
    'AP_Relay',
    'AP_ServoRelayEvents',
    'AP_Volz_Protocol',
    'AP_SBusOut',
    'AP_IOMCU',
    'AP_Parachute',
    'AP_RAMTRON',
    'AP_RCProtocol',
    'AP_Radio',
    'AP_TempCalibration',
    'AP_VisualOdom',
    'AP_BLHeli',
    'AP_ROMFS',
    'AP_Proximity',
    'AP_Gripper',
    'AP_RTC',
    'AC_Sprayer',
    'AC_Fence',
    'AC_Avoidance',
    'AP_LandingGear',
    'AP_RobotisServo',
    'AP_NMEA_Output',
    'AP_OSD',
    'AP_Filesystem',
    'AP_ADSB',
    'AP_ADSB/sagetech-sdk',
    'AC_PID',
    'AP_SerialLED',
    'AP_EFI',
    'AP_Hott_Telem',
    'AP_ESC_Telem',
    'AP_Servo_Telem',
    'AP_Stats',
    'AP_GyroFFT',
    'AP_RCTelemetry',
    'AP_Generator',
    'AP_MSP',
    'AP_OLC',
    'AP_WheelEncoder',
    'AP_ExternalAHRS',
    'AP_VideoTX',
    'AP_FETtecOneWire',
    'AP_TemperatureSensor',
    'AP_Torqeedo',
    'AP_CustomRotations',
    'AP_AIS',
    'AP_OpenDroneID',
    'AP_CheckFirmware',
    'AP_ExternalControl',
    'AP_JSON',
    'AP_Beacon',
    'AP_Arming',
    'AP_RCMapper',
    'AP_MultiHeap',
    'AP_Follow',
]

def get_legacy_defines(sketch_name, bld):
    # If we are building heli, we adjust the build directory define so that 
    # we do not need to actually split heli and copter directories
    if bld.cmd == 'heli' or 'heli' in bld.targets:
        return [
        'APM_BUILD_DIRECTORY=APM_BUILD_Heli',
        'AP_BUILD_TARGET_NAME="' + sketch_name + '"',
        ]

    return [
        'APM_BUILD_DIRECTORY=APM_BUILD_' + sketch_name,
        'AP_BUILD_TARGET_NAME="' + sketch_name + '"',
    ]

def set_double_precision_flags(flags):
    # set up flags for double precision files:
    # * remove all single precision constant flags
    # * set define to allow double precision math in AP headers

    flags = flags[:] # copy the list to avoid affecting other builds

    # remove GCC and clang single precision constant flags
    for opt in ('-fsingle-precision-constant', '-cl-single-precision-constant'):
        while True: # might have multiple copies from different sources
            try:
                flags.remove(opt)
            except ValueError:
                break
    flags.append("-DAP_MATH_ALLOW_DOUBLE_FUNCTIONS=1")

    return flags

IGNORED_AP_LIBRARIES = [
    'doc',
    'AP_Scripting', # this gets explicitly included when it is needed and should otherwise never be globbed in
]


def ap_autoconfigure(execute_method):
    """
    Decorator that enables context commands to run *configure* as needed.
    """
    def execute(self):
        """
        Wraps :py:func:`waflib.Context.Context.execute` on the context class
        """
        if 'tools/' in self.targets:
            raise Errors.WafError('\"tools\" name has been replaced with \"tool\" for build please use that!')
        if not Configure.autoconfig:
            return execute_method(self)

        # Disable autoconfig so waf's version doesn't run (and don't end up on loop of bad configure)
        Configure.autoconfig = False

        if self.variant == '':
            raise Errors.WafError('The project is badly configured: run "waf configure" again!')

        env = ConfigSet.ConfigSet()
        do_config = False

        try:
            p = os.path.join(Context.out_dir, Build.CACHE_DIR, self.variant + Build.CACHE_SUFFIX)
            env.load(p)
        except EnvironmentError:
            raise Errors.WafError('The project is not configured for board {0}: run "waf configure --board {0} [...]" first!'.format(self.variant))

        lock_env = ConfigSet.ConfigSet()

        try:
            lock_env.load(os.path.join(Context.top_dir, Options.lockfile))
        except EnvironmentError:
            Logs.warn('Configuring the project')
            do_config = True
        else:
            if lock_env.run_dir != Context.run_dir:
                do_config = True
            else:
                h = 0

                for f in env.CONFIGURE_FILES:
                    try:
                        h = Utils.h_list((h, Utils.readf(f, 'rb')))
                    except EnvironmentError:
                        do_config = True
                        break
                else:
                    do_config = h != env.CONFIGURE_HASH

        if do_config:
            cmd = lock_env.config_cmd or 'configure'
            tmp = Options.options.__dict__

            if env.OPTIONS and sorted(env.OPTIONS.keys()) == sorted(tmp.keys()):
                Options.options.__dict__ = env.OPTIONS
            else:
                raise Errors.WafError('The project configure options have changed: run "waf configure" again!')

            try:
                run_command(cmd)
            finally:
                Options.options.__dict__ = tmp

            run_command(self.cmd)
        else:
            return execute_method(self)

    return execute

def ap_configure_post_recurse():
    post_recurse_orig = Configure.ConfigurationContext.post_recurse

    def post_recurse(self, node):
        post_recurse_orig(self, node)

        self.all_envs[self.variant].CONFIGURE_FILES = self.files
        self.all_envs[self.variant].CONFIGURE_HASH = self.hash

    return post_recurse

@conf
def ap_get_all_libraries(bld):
    if bld.env.BOOTLOADER:
        # we don't need the full set of libraries for the bootloader build
        return ['AP_HAL']
    libraries = []
    for lib_node in bld.srcnode.ant_glob('libraries/*', dir=True, src=False):
        name = lib_node.name
        if name in IGNORED_AP_LIBRARIES:
            continue
        if name.startswith('AP_HAL'):
            continue
        if name == 'SITL':
            continue
        libraries.append(name)
    libraries.extend(['AP_HAL', 'AP_HAL_Empty'])
    libraries.append('AP_PiccoloCAN/piccolo_protocol')
    return libraries

@conf
def ap_common_vehicle_libraries(bld):
    libraries = COMMON_VEHICLE_DEPENDENT_LIBRARIES

    if bld.env.with_can or bld.env.HAL_NUM_CAN_IFACES:
        libraries.extend(COMMON_VEHICLE_DEPENDENT_CAN_LIBRARIES)

    return libraries

_grouped_programs = {}


class upload_fw_blueos(Task.Task):
    def run(self):
        # this is rarely used, so we import requests here to avoid the overhead
        import requests
        binary_path = self.inputs[0].abspath()
        # check if .apj file exists for chibios builds
        if Path(binary_path + ".apj").exists():
            binary_path = binary_path + ".apj"
        bld = self.generator.bld
        board = bld.bldnode.name.capitalize()
        print(f"Uploading {binary_path} to BlueOS at {bld.options.upload_blueos} for board {board}")
        url = f'{bld.options.upload_blueos}/ardupilot-manager/v1.0/install_firmware_from_file?board_name={board}'
        files = {
          'binary': open(binary_path, 'rb')
        }
        response = requests.post(url, files=files, verify=False)
        if response.status_code != 200:
            raise Errors.WafError(f"Failed to upload firmware to BlueOS: {response.status_code}: {response.text}")
        print("Upload complete")

    def keyword(self):
          return "Uploading to BlueOS"

class check_elf_symbols(Task.Task):
    color='CYAN'
    always_run = True
    def keyword(self):
        return "checking symbols"

    def run(self):
        '''
        check for disallowed symbols in elf file, such as C++ exceptions
        '''
        elfpath = self.inputs[0].abspath()

        if not self.env.CHECK_SYMBOLS:
            # checking symbols disabled on this build
            return

        if not self.env.vehicle_binary or self.env.SIM_ENABLED:
            # we only want to check symbols for vehicle binaries, allowing examples
            # to use C++ exceptions. We also allow them in simulator builds
            return

        # we use string find on these symbols, so this catches all types of throw
        # calls this should catch all uses of exceptions unless the compiler
        # manages to inline them
        blacklist = ['std::__throw',
                     'operator new[](unsigned int)',
                     'operator new[](unsigned long)',
                     'operator new(unsigned int)',
                     'operator new(unsigned long)']

        nmout = subprocess.getoutput("%s -C %s" % (self.env.get_flat('NM'), elfpath))
        for b in blacklist:
            if nmout.find(b) != -1:
                raise Errors.WafError("Disallowed symbol in %s: %s" % (elfpath, b))


@feature('post_link')
@after_method('process_source')
def post_link(self):
    '''
    setup tasks to run after link stage
    '''
    self.link_task.always_run = True

    link_output = self.link_task.outputs[0]

    check_elf_task = self.create_task('check_elf_symbols', src=link_output)
    check_elf_task.set_run_after(self.link_task)
    if self.bld.options.upload_blueos and self.env["BOARD_CLASS"] == "LINUX":
        _upload_task = self.create_task('upload_fw_blueos', src=link_output)
        _upload_task.set_run_after(self.link_task)

@conf
def ap_program(bld,
               program_groups='bin',
               program_dir=None,
               use_legacy_defines=True,
               program_name=None,
               vehicle_binary=True,
               **kw):
    if 'target' in kw:
        bld.fatal('Do not pass target for program')
    if 'defines' not in kw:
        kw['defines'] = []
    if 'source' not in kw:
        kw['source'] = bld.path.ant_glob(SOURCE_EXTS)

    if not program_name:
        program_name = bld.path.name

    if use_legacy_defines:
        kw['defines'].extend(get_legacy_defines(bld.path.name, bld))

    kw['features'] = kw.get('features', []) + bld.env.AP_PROGRAM_FEATURES + ['post_link']

    program_groups = Utils.to_list(program_groups)

    if not program_dir:
        program_dir = program_groups[0]

    name = os.path.join(program_dir, program_name)

    tg_constructor = bld.program
    if bld.env.AP_PROGRAM_AS_STLIB:
        tg_constructor = bld.stlib
    else:
        if bld.env.STATIC_LINKING:
            kw['features'].append('static_linking')


    tg = tg_constructor(
        target='#%s' % name,
        name=name,
        program_name=program_name,
        program_dir=program_dir,
        **kw
    )

    tg.env.vehicle_binary = vehicle_binary

    if 'use' in kw and bld.env.STATIC_LINKING:
        # ensure we link against vehicle library
        tg.env.STLIB += [kw['use']]

    for group in program_groups:
        _grouped_programs.setdefault(group, {}).update({tg.name : tg})

    return tg


@conf
def ap_example(bld, **kw):
    kw['program_groups'] = 'examples'
    ap_program(bld, use_legacy_defines=False, vehicle_binary=False, **kw)

def unique_list(items):
    '''remove duplicate elements from a list while maintaining ordering'''
    return list(OrderedDict.fromkeys(items))

@conf
def ap_stlib(bld, **kw):
    if 'name' not in kw:
        bld.fatal('Missing name for ap_stlib')
    if 'ap_vehicle' not in kw:
        bld.fatal('Missing ap_vehicle for ap_stlib')
    if 'ap_libraries' not in kw:
        bld.fatal('Missing ap_libraries for ap_stlib')

    kw['ap_libraries'] = unique_list(kw['ap_libraries'] + bld.env.AP_LIBRARIES)
    for l in kw['ap_libraries']:
        bld.ap_library(l, kw['ap_vehicle'])

    if 'dynamic_source' not in kw:
        kw['dynamic_source'] = 'modules/DroneCAN/libcanard/dsdlc_generated/src/**.c'

    kw['features'] = kw.get('features', []) + ['cxx', 'cxxstlib']
    kw['target'] = kw['name']
    kw['source'] = []

    bld.stlib(**kw)

_created_program_dirs = set()
@feature('cxxstlib', 'cxxprogram')
@before_method('process_rule')
def ap_create_program_dir(self):
    if not hasattr(self, 'program_dir'):
        return
    if self.program_dir in _created_program_dirs:
        return
    self.bld.bldnode.make_node(self.program_dir).mkdir()
    _created_program_dirs.add(self.program_dir)

@feature('cxxstlib')
@before_method('process_rule')
def ap_stlib_target(self):
    if self.target.startswith('#'):
        self.target = self.target[1:]
    self.target = '#%s' % os.path.join('lib', self.target)

@conf
def ap_find_tests(bld, use=[], DOUBLE_PRECISION_SOURCES=[]):
    if not bld.env.HAS_GTEST:
        return

    features = []
    if bld.cmd == 'check':
        features.append('test')

    use = Utils.to_list(use)
    use.append('GTEST')

    includes = [bld.srcnode.abspath() + '/tests/']

    for f in bld.path.ant_glob(incl='*.cpp'):
        t = ap_program(
            bld,
            features=features,
            includes=includes,
            source=[f],
            use=use,
            program_name=f.change_ext('').name,
            program_groups='tests',
            use_legacy_defines=False,
            vehicle_binary=False,
            cxxflags=['-Wno-undef'],
        )
        filename = os.path.basename(f.abspath())
        if filename in DOUBLE_PRECISION_SOURCES:
            t.env.CXXFLAGS = set_double_precision_flags(t.env.CXXFLAGS)

_versions = []

@conf
def ap_version_append_str(ctx, k, v, consistent_v=None):
    if ctx.env.CONSISTENT_BUILDS and consistent_v is not None:
        v = consistent_v # override with consistent value
    else:
        v = os.environ.get(k, v) # use v unless defined in environment

    ctx.env['AP_VERSION_ITEMS'] += [(k, f'"{v}"')]

@conf
def ap_version_append_int(ctx, k, v, consistent_v=None):
    if ctx.env.CONSISTENT_BUILDS and consistent_v is not None:
        v = consistent_v # override with consistent value
    else:
        v = os.environ.get(k, v) # use v unless defined in environment

    ctx.env['AP_VERSION_ITEMS'] += [(k, f'{v}')]

@conf
def write_version_header(ctx, tgt):
    with open(tgt, 'w') as f:
        print(
'''// auto-generated header, do not edit

#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error ap_version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif
''', file=f)

        for k, v in ctx.env['AP_VERSION_ITEMS']:
            print('#define {} {}'.format(k, v), file=f)

@conf
def ap_find_benchmarks(bld, use=[]):
    if not bld.env.HAS_GBENCHMARK:
        return

    includes = [bld.srcnode.abspath() + '/benchmarks/']
    to_remove = '-Werror=suggest-override'
    if to_remove in bld.env.CXXFLAGS:
        need_remove = True
    else:
        need_remove = False
    if need_remove:
        while to_remove in bld.env.CXXFLAGS:
            bld.env.CXXFLAGS.remove(to_remove)

    for f in bld.path.ant_glob(incl='*.cpp'):
        ap_program(
            bld,
            features=['gbenchmark'],
            includes=includes,
            source=[f],
            use=use,
            vehicle_binary=False,
            program_name=f.change_ext('').name,
            program_groups='benchmarks',
            use_legacy_defines=False,
        )

def test_summary(bld):
    from io import BytesIO
    import sys

    if not hasattr(bld, 'utest_results'):
        Logs.info('check: no test run')
        return

    fails = []

    for filename, exit_code, out, err in bld.utest_results:
        Logs.pprint('GREEN' if exit_code == 0 else 'YELLOW',
                    '    %s' % filename,
                    'returned %d' % exit_code)

        if exit_code != 0:
            fails.append(filename)
        elif not bld.options.check_verbose:
            continue

        if len(out):
            buf = BytesIO(out)
            for line in buf:
                print("    OUT: %s" % line.decode(), end='', file=sys.stderr)
            print()

        if len(err):
            buf = BytesIO(err)
            for line in buf:
                print("    ERR: %s" % line.decode(), end='', file=sys.stderr)
            print()

    if not fails:
        Logs.info('check: All %u tests passed!' % len(bld.utest_results))
        return

    Logs.error('check: %u of %u tests failed' %
               (len(fails), len(bld.utest_results)))

    for filename in fails:
        Logs.error('    %s' % filename)

    bld.fatal('check: some tests failed')

_build_commands = {}

def _process_build_command(bld):
    if bld.cmd not in _build_commands:
        return

    params = _build_commands[bld.cmd]

    targets = params['targets']
    if targets:
        if bld.targets:
            bld.targets += ',' + targets
        else:
            bld.targets = targets

    program_group_list = Utils.to_list(params['program_group_list'])
    bld.options.program_group.extend(program_group_list)

def build_command(name,
                   targets=None,
                   program_group_list=[],
                   doc='build shortcut'):
    _build_commands[name] = dict(
        targets=targets,
        program_group_list=program_group_list,
    )

    class context_class(Build.BuildContext):
        cmd = name
    context_class.__doc__ = doc

def _select_programs_from_group(bld):
    groups = bld.options.program_group
    if not groups:
        if bld.targets:
            groups = []
        else:
            groups = ['bin']

    possible_groups = list(_grouped_programs.keys())
    possible_groups.remove('bin')       # Remove `bin` so as not to duplicate all items in bin
    if 'all' in groups:
        groups = possible_groups

    for group in groups:
        if group not in _grouped_programs:
            bld.fatal(f'Group {group} not found, possible groups: {possible_groups}')

        target_names = _grouped_programs[group].keys()

        for name in target_names:
            if bld.targets:
                bld.targets += ',' + name
            else:
                bld.targets = name

def options(opt):
    opt.ap_groups = {
        'configure': opt.add_option_group('Ardupilot configure options'),
        'linux': opt.add_option_group('Linux boards configure options'),
        'build': opt.add_option_group('Ardupilot build options'),
        'check': opt.add_option_group('Ardupilot check options'),
        'clean': opt.add_option_group('Ardupilot clean options'),
    }

    g = opt.ap_groups['build']

    g.add_option('--program-group',
        action='append',
        default=[],
        help='''Select all programs that go in <PROGRAM_GROUP>/ for the build.
Example: `waf --program-group examples` builds all examples. The
special group "all" selects all programs.
''')

    g.add_option('--upload',
        action='store_true',
        help='''Upload applicable targets to a connected device. Not all
platforms may support this. Example: `waf copter --upload` means "build
arducopter and upload it to my board".
''')

    g.add_option('--upload-port',
        action='store',
        dest='upload_port',
        default=None,
        help='''Specify the port to be used with the --upload option. For example a port of /dev/ttyS10 indicates that serial port 10 should be used.
''')

    g.add_option('--upload-blueos',
        action='store',
        dest='upload_blueos',
        default=None,
        help='''Automatically upload to a BlueOS device. The argument is the url for the device. http://blueos.local for example.
''')

    g.add_option('--upload-force',
        action='store_true',
        help='''Override board type check and continue loading. Same as using uploader.py --force.
''')

    g.add_option('--define',
        action='append',
        help='Add C++ define to build.')

    g = opt.ap_groups['check']

    g.add_option('--check-verbose',
        action='store_true',
        help='Output all test programs.')

    g = opt.ap_groups['clean']

    g.add_option('--asan',
        action='store_true',
        help='''Build using the macOS clang Address Sanitizer. In order to run with
Address Sanitizer support llvm-symbolizer is required to be on the PATH.
This option is only supported on macOS versions of clang.
''')

    g.add_option('--ubsan',
        action='store_true',
        help='''Build using the gcc undefined behaviour sanitizer''')

    g.add_option('--ubsan-abort',
        action='store_true',
        help='''Build using the gcc undefined behaviour sanitizer and abort on error''')
    
def build(bld):
    bld.add_pre_fun(_process_build_command)
    bld.add_pre_fun(_select_programs_from_group)

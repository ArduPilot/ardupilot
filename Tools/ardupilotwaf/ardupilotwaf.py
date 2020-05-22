#!/usr/bin/env python
# encoding: utf-8

from __future__ import print_function
from waflib import Build, ConfigSet, Configure, Context, Errors, Logs, Options, Utils
from waflib.Configure import conf
from waflib.Scripting import run_command
from waflib.TaskGen import before_method, feature
import os.path, os
from collections import OrderedDict

import ap_persistent

SOURCE_EXTS = [
    '*.S',
    '*.c',
    '*.cpp',
]

COMMON_VEHICLE_DEPENDENT_LIBRARIES = [
    'AP_AccelCal',
    'AP_ADC',
    'AP_AHRS',
    'AP_Airspeed',
    'AP_Baro',
    'AP_BattMonitor',
    'AP_BoardConfig',
    'AP_Common',
    'AP_Compass',
    'AP_Declination',
    'AP_GPS',
    'AP_HAL',
    'AP_HAL_Empty',
    'AP_InertialSensor',
    'AP_Math',
    'AP_Mission',
    'AP_NavEKF',
    'AP_NavEKF2',
    'AP_NavEKF3',
    'AP_Notify',
    'AP_OpticalFlow',
    'AP_Param',
    'AP_Rally',
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
    'AP_Frsky_Telem',
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
    'AP_ToshibaCAN',
    'AP_NMEA_Output',
    'AP_Filesystem',
    'AP_ADSB',
    'AC_PID',
    'AP_SerialLED',
    'AP_Hott_Telem',
]

def get_legacy_defines(sketch_name):
    return [
        'APM_BUILD_DIRECTORY=APM_BUILD_' + sketch_name,
        'SKETCH="' + sketch_name + '"',
        'SKETCHNAME="' + sketch_name + '"',
    ]

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
    return libraries

@conf
def ap_common_vehicle_libraries(bld):
    libraries = COMMON_VEHICLE_DEPENDENT_LIBRARIES

    if bld.env.DEST_BINFMT == 'pe':
        libraries += [
            'AC_Fence',
            'AC_AttitudeControl',
        ]

    return libraries

_grouped_programs = {}

@conf
def ap_program(bld,
               program_groups='bin',
               program_dir=None,
               use_legacy_defines=True,
               program_name=None,
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
        kw['defines'].extend(get_legacy_defines(bld.path.name))

    kw['cxxflags'] = kw.get('cxxflags', []) + ['-include', 'ap_config.h']
    kw['features'] = kw.get('features', []) + bld.env.AP_PROGRAM_FEATURES

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
    if 'use' in kw and bld.env.STATIC_LINKING:
        # ensure we link against vehicle library
        tg.env.STLIB += [kw['use']]

    for group in program_groups:
        _grouped_programs.setdefault(group, []).append(tg)

@conf
def ap_example(bld, **kw):
    kw['program_groups'] = 'examples'
    ap_program(bld, use_legacy_defines=False, **kw)

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
def ap_find_tests(bld, use=[]):
    if not bld.env.HAS_GTEST:
        return

    features = []
    if bld.cmd == 'check':
        features.append('test')

    use = Utils.to_list(use)
    use.append('GTEST')

    includes = [bld.srcnode.abspath() + '/tests/']

    for f in bld.path.ant_glob(incl='*.cpp'):
        ap_program(
            bld,
            features=features,
            includes=includes,
            source=[f],
            use=use,
            program_name=f.change_ext('').name,
            program_groups='tests',
            use_legacy_defines=False,
            cxxflags=['-Wno-undef'],
        )

_versions = []

@conf
def ap_version_append_str(ctx, k, v):
    ctx.env['AP_VERSION_ITEMS'] += [(k, '"{}"'.format(os.environ.get(k, v)))]

@conf
def ap_version_append_int(ctx, k, v):
    ctx.env['AP_VERSION_ITEMS'] += [(k,v)]

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

    for f in bld.path.ant_glob(incl='*.cpp'):
        ap_program(
            bld,
            features=['gbenchmark'],
            includes=includes,
            source=[f],
            use=use,
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

    if 'all' in groups:
        groups = _grouped_programs.keys()

    for group in groups:
        if group not in _grouped_programs:
            bld.fatal('Group %s not found' % group)

        tg = _grouped_programs[group][0]
        if bld.targets:
            bld.targets += ',' + tg.name
        else:
            bld.targets = tg.name

        for tg in _grouped_programs[group][1:]:
            bld.targets += ',' + tg.name

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
        help='''Specify the port to be used with the --upload option. For example a port of /dev/ttyS10 indicates that serial port 10 shuld be used.
''')

    g = opt.ap_groups['check']

    g.add_option('--check-verbose',
        action='store_true',
        help='Output all test programs.')

    g = opt.ap_groups['clean']

    g.add_option('--clean-all-sigs',
        action='store_true',
        help='''Clean signatures for all tasks. By default, tasks that scan for
implicit dependencies (like the compilation tasks) keep the dependency
information across clean commands, so that that information is changed
only when really necessary. Also, some tasks that don't really produce
files persist their signature. This option avoids that behavior when
cleaning the build.
''')

def build(bld):
    bld.add_pre_fun(_process_build_command)
    bld.add_pre_fun(_select_programs_from_group)

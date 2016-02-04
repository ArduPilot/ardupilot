#!/usr/bin/env python
# encoding: utf-8

from __future__ import print_function
from waflib import Logs, Options, Utils
from waflib.Build import BuildContext
from waflib.Configure import conf
import os.path

SOURCE_EXTS = [
    '*.S',
    '*.c',
    '*.cpp',
]

UTILITY_SOURCE_EXTS = [ 'utility/' + glob for glob in SOURCE_EXTS ]

COMMON_VEHICLE_DEPENDENT_LIBRARIES = [
    'AP_AccelCal',
    'AP_ADC',
    'AP_AHRS',
    'AP_Airspeed',
    'AP_Baro',
    'AP_BattMonitor',
    'AP_BoardConfig',
    'AP_Buffer',
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
    'AP_Notify',
    'AP_OpticalFlow',
    'AP_Param',
    'AP_Rally',
    'AP_RangeFinder',
    'AP_Scheduler',
    'AP_SerialManager',
    'AP_Terrain',
    'AP_Vehicle',
    'DataFlash',
    'Filter',
    'GCS_MAVLink',
    'RC_Channel',
    'SITL',
    'StorageManager',
]

def _get_legacy_defines(sketch_name):
    return [
        'APM_BUILD_DIRECTORY=APM_BUILD_' + sketch_name,
        'SKETCH="' + sketch_name + '"',
        'SKETCHNAME="' + sketch_name + '"',
    ]

IGNORED_AP_LIBRARIES = [
    'doc',
    'GCS_Console',
]

@conf
def ap_get_all_libraries(bld):
    libraries = []
    for lib_node in bld.srcnode.ant_glob('libraries/*', dir=True):
        name = lib_node.name
        if name in IGNORED_AP_LIBRARIES:
            continue
        if name.startswith('AP_HAL'):
            continue
        libraries.append(name)
    libraries.extend(['AP_HAL', 'AP_HAL_Empty'])
    return libraries

@conf
def ap_common_vehicle_libraries(bld):
    return COMMON_VEHICLE_DEPENDENT_LIBRARIES

_grouped_programs = {}

@conf
def ap_program(bld, program_group='bin',
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
        kw['defines'].extend(_get_legacy_defines(bld.path.name))

    kw['features'] = common_features(bld) + kw.get('features', [])

    name = os.path.join(program_group, program_name)
    target = bld.bldnode.find_or_declare(name)

    tg = bld.program(
        target=target,
        name=name,
        **kw
    )
    _grouped_programs.setdefault(program_group, []).append(tg)

@conf
def ap_example(bld, **kw):
    kw['program_group'] = 'examples'
    ap_program(bld, **kw)

# NOTE: Code in libraries/ is compiled multiple times. So ensure each
# compilation is independent by providing different index for each.
# The need for this should disappear when libraries change to be
# independent of vehicle type.
LAST_IDX = 0

def _get_next_idx():
    global LAST_IDX
    LAST_IDX += 1
    return LAST_IDX

def common_features(bld):
    features = []
    if bld.env.STATIC_LINKING:
        features.append('static_linking')
    return features

@conf
def ap_stlib(bld, **kw):
    if 'name' not in kw:
        bld.fatal('Missing name for ap_stlib')
    if 'vehicle' not in kw:
        bld.fatal('Missing vehicle for ap_stlib')
    if 'libraries' not in kw:
        bld.fatal('Missing libraries for ap_stlib')

    sources = []
    libraries = kw['libraries'] + bld.env.AP_LIBRARIES

    for lib_name in libraries:
        lib_node = bld.srcnode.find_dir('libraries/' + lib_name)
        if lib_node is None:
            bld.fatal('Could not find library ' + lib_name)
        lib_sources = lib_node.ant_glob(SOURCE_EXTS + UTILITY_SOURCE_EXTS)
        sources.extend(lib_sources)

    kw['source'] = sources
    kw['target'] = kw['name']
    kw['defines'] = _get_legacy_defines(kw['vehicle'])
    kw['idx'] = _get_next_idx()

    bld.stlib(**kw)

@conf
def ap_find_tests(bld, use=[]):
    if not bld.env.HAS_GTEST:
        return

    features = common_features(bld)
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
            program_group='tests',
            use_legacy_defines=False,
        )

@conf
def ap_find_benchmarks(bld, use=[]):
    if not bld.env.HAS_GBENCHMARK:
        return

    includes = [bld.srcnode.abspath() + '/benchmarks/']

    for f in bld.path.ant_glob(incl='*.cpp'):
        ap_program(
            bld,
            features=common_features(bld) + ['gbenchmark'],
            includes=includes,
            source=[f],
            use=use,
            program_name=f.change_ext('').name,
            program_group='benchmarks',
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

    class context_class(BuildContext):
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
    g = opt.add_option_group('Ardupilot build options')
    g.add_option('--program-group',
        action='append',
        default=[],
        help='Select all programs that go in <PROGRAM_GROUP>/ for the ' +
             'build. Example: `waf --program-group examples` builds all ' +
             'examples. The special group "all" selects all programs.',
    )

def build(bld):
    global LAST_IDX
    # FIXME: This is done to prevent same task generators being created with
    # different idx when build() is called multiple times (e.g. waf bin tests).
    # Ideally, task generators should be created just once.
    LAST_IDX = 0
    bld.add_pre_fun(_process_build_command)
    bld.add_pre_fun(_select_programs_from_group)

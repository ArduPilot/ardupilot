#!/usr/bin/env python
# encoding: utf-8

SOURCE_EXTS = [
    '*.S',
    '*.c',
    '*.cpp',
]

UTILITY_SOURCE_EXTS = [ 'utility/' + glob for glob in SOURCE_EXTS ]

# TODO: Once HAL patches get in, need to filter out the HAL based
# on the bld.env.BOARD.
COMMON_VEHICLE_DEPENDENT_LIBRARIES = [
    'AP_ADC',
    'AP_ADC_AnalogSource',
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
    'AP_Progmem',
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

def _get_legacy_defines(name):
    return [
        'APM_BUILD_DIRECTORY=' + name,
        'SKETCH="' + name + '"',
        'SKETCHNAME="' + name + '"',
    ]

IGNORED_AP_LIBRARIES = [
    'doc',
    'AP_Limits',
    'GCS_Console',
]

def get_all_libraries(bld):
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

def program(bld, **kw):
    if 'target' in kw:
        bld.fatal('Do not pass target for program')
    if 'defines' not in kw:
        kw['defines'] = []
    if 'source' not in kw:
        kw['source'] = bld.path.ant_glob(SOURCE_EXTS)

    name = bld.path.name
    kw['defines'].extend(_get_legacy_defines(name))

    target = bld.bldnode.make_node(name + '.' + bld.env.BOARD)
    bld.program(
        target=target,
        **kw
    )

# NOTE: Code in libraries/ is compiled multiple times. So ensure each
# compilation is independent by providing different index for each.
# The need for this should disappear when libraries change to be
# independent of vehicle type.
LAST_IDX = 0

def _get_next_idx():
    global LAST_IDX
    LAST_IDX += 1
    return LAST_IDX

def vehicle_stlib(bld, **kw):
    if 'name' not in kw:
        bld.fatal('Missing name for vehicle_stlib')
    if 'vehicle' not in kw:
        bld.fatal('Missing vehicle for vehicle_stlib')
    if 'libraries' not in kw:
        bld.fatal('Missing libraries for vehicle_stlib')

    sources = []
    libraries = kw['libraries'] + bld.env.AP_LIBRARIES

    for lib_name in libraries:
        lib_node = bld.srcnode.find_dir('libraries/' + lib_name)
        if lib_node is None:
            bld.fatal('Could not find library ' + lib_name)
        lib_sources = lib_node.ant_glob(SOURCE_EXTS + UTILITY_SOURCE_EXTS)
        sources.extend(lib_sources)

    name = kw['name']
    vehicle = kw['vehicle']

    bld.stlib(
        source=sources,
        target=name,
        defines=_get_legacy_defines(vehicle),
        idx=_get_next_idx(),
    )

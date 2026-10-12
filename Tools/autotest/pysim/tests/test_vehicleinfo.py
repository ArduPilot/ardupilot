"""Checks for vehicleinfo.json and vehicleinfo.py.

sim_vehicle.py and autotest read vehicleinfo.json through vehicleinfo.py,
and SITL embeds it in ROMFS and looks up a model's default parameter files
in it at startup (SITL_State::resolve_defaults_from_romfs).  Check that it
is strict JSON, that it has the shape they rely on and that
vehicleinfo_schema.md documents, that the files it names exist and, for
SITL's defaults, are embedded in ROMFS, and that VehicleInfo looks frames
up as expected.

Run from the ArduPilot root:
    python3 -m pytest Tools/autotest/pysim/tests/test_vehicleinfo.py

AP_FLAKE8_CLEAN
"""

import fnmatch
import json
import os
import re

from argparse import Namespace

import pytest

from Tools.autotest.pysim import vehicleinfo

PYSIM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUTOTEST = os.path.dirname(PYSIM)
ROOT = os.path.dirname(os.path.dirname(AUTOTEST))
VEHICLEINFO = os.path.join(PYSIM, 'vehicleinfo.json')
SCHEMA = os.path.join(PYSIM, 'vehicleinfo_schema.md')
SCRIPTING_EXAMPLES = os.path.join(ROOT, 'libraries', 'AP_Scripting', 'examples')

# the files SITL embeds in ROMFS from these Tools/autotest directories, see
# SITLBoard.configure_env() in Tools/ardupilotwaf/boards.py
SITL_ROMFS_PATTERNS = {
    'default_params': ['*.parm'],
    'models': ['*.json', '*.parm', 'mt11_*.h264'],
}

# the types of the frame fields, as documented in vehicleinfo_schema.md
FIELD_TYPES = {
    'waf_target': (str,),
    'model': (str,),
    'configure_target': (str,),
    'extra_mavlink_cmds': (str,),
    'frame_example_script': (str,),
    'periph_board': (str,),
    'external': (bool,),
    'configure_args': (list,),
    'periph_extra_args': (list,),
    'default_params_filename': (str, list),
    'periph_params_filename': (list,),
}


def _reject_constant(name):
    # AP_JSON, which SITL uses, rejects NaN and Infinity
    raise ValueError("not JSON: %s" % name)


def _reject_duplicates(pairs):
    keys = [key for key, _ in pairs]
    duplicates = sorted({key for key in keys if keys.count(key) > 1})
    if duplicates:
        raise ValueError("duplicate keys: %s" % duplicates)
    return dict(pairs)


@pytest.fixture(scope='module')
def info():
    with open(VEHICLEINFO) as f:
        return json.load(f, parse_constant=_reject_constant, object_pairs_hook=_reject_duplicates)


@pytest.fixture(scope='module')
def schema():
    with open(SCHEMA) as f:
        return f.read()


def documented_fields(schema):
    '''the field names in the "Per-frame fields" table'''
    section = schema.split('## Per-frame fields', 1)[1].split('\n## ', 1)[0]
    return set(re.findall(r'^\| `([^`]+)` \|', section, re.MULTILINE))


def all_frames(info):
    for vehicle_name, vehicle in info.items():
        for frame_name, frame in vehicle['frames'].items():
            yield "%s %s" % (vehicle_name, frame_name), frame


def file_names(value):
    '''a *_params_filename value: one name or a list of them'''
    return [value] if isinstance(value, str) else value


# the JSON file

def test_vehicles_have_frames(info):
    assert isinstance(info, dict)
    for name, vehicle in info.items():
        assert isinstance(vehicle, dict), name
        assert isinstance(vehicle.get('frames'), dict), name
        assert vehicle['frames'], name


def test_default_frame_exists(info):
    for name, vehicle in info.items():
        # sim_vehicle.py uses it when started without -f; periph boards
        # are only started by the vehicles that name them
        if name.startswith('sitl_periph_'):
            continue
        assert 'default_frame' in vehicle, name
        default_frame = vehicle['default_frame']
        assert isinstance(default_frame, str), name
        assert default_frame in vehicle['frames'], name


def test_frames_have_a_waf_target(info):
    # sim_vehicle.py builds it, and autotest starts the periph boards from
    # it; only the calibration frames, a MAVProxy mode, have none
    for where, frame in all_frames(info):
        if where.endswith(' calibration'):
            continue
        assert isinstance(frame.get('waf_target'), str), where


def test_frame_fields_are_documented_and_typed(info, schema):
    documented = documented_fields(schema)
    # keep this test's types in step with the documented fields
    assert documented == set(FIELD_TYPES), "update FIELD_TYPES to match vehicleinfo_schema.md"
    for where, frame in all_frames(info):
        assert isinstance(frame, dict), where
        for key, value in frame.items():
            assert key in documented, "%s: %s is not in vehicleinfo_schema.md" % (where, key)
            assert isinstance(value, FIELD_TYPES[key]), "%s: %s" % (where, key)
            if isinstance(value, list):
                assert all(isinstance(item, str) for item in value), "%s: %s" % (where, key)


def test_vehicles_are_documented(info, schema):
    # the "Top-level keys are vehicle names" paragraph
    paragraph = schema.split('Top-level keys are vehicle names:', 1)[1].split('\n\n', 1)[0]
    documented = set(re.findall(r'`([^`]+)`', paragraph))
    for name in info:
        assert name in documented, "%s is not in vehicleinfo_schema.md" % name


def test_periph_board_exists(info):
    # sim_vehicle.py runs the peripheral with the board's "universal" frame
    for where, frame in all_frames(info):
        if 'periph_board' in frame:
            board = info.get(frame['periph_board'])
            assert board is not None, where
            assert 'universal' in board['frames'], where


def test_params_files_exist(info):
    # sim_vehicle.py and autotest load them from Tools/autotest
    for where, frame in all_frames(info):
        for key in 'default_params_filename', 'periph_params_filename':
            for name in file_names(frame.get(key, [])):
                assert os.path.isfile(os.path.join(AUTOTEST, name)), "%s: %s" % (where, name)


def test_default_params_are_embedded_in_sitl_romfs(info):
    # SITL loads them as @ROMFS/<name>, so each must be one SITL embeds
    for where, frame in all_frames(info):
        for name in file_names(frame.get('default_params_filename', [])):
            directory, _, filename = name.partition('/')
            patterns = SITL_ROMFS_PATTERNS.get(directory, [])
            # boards.py embeds only the top level of each directory
            assert '/' not in filename and any(fnmatch.fnmatch(filename, p) for p in patterns), \
                "%s: %s is not embedded in SITL's ROMFS" % (where, name)


def test_frame_example_scripts_exist(info):
    for where, frame in all_frames(info):
        if 'frame_example_script' in frame:
            path = os.path.join(SCRIPTING_EXAMPLES, frame['frame_example_script'])
            assert os.path.isfile(path), where


# vehicleinfo.py

def no_overrides():
    return Namespace(model=None, build_target=None)


def test_loads_the_json_file(info):
    assert vehicleinfo.VehicleInfo().options == info


def test_default_frame_and_waf_target(info):
    vinfo = vehicleinfo.VehicleInfo()
    for name, vehicle in info.items():
        if 'default_frame' not in vehicle:
            continue
        assert vinfo.default_frame(name) == vehicle['default_frame']
        assert vinfo.default_waf_target(name) == vehicle['frames'][vehicle['default_frame']]['waf_target']


def test_options_for_every_frame(info):
    vinfo = vehicleinfo.VehicleInfo()
    for name, vehicle in info.items():
        if 'default_frame' not in vehicle:
            continue
        for frame_name, frame in vehicle['frames'].items():
            ret = vinfo.options_for_frame(frame_name, name, no_overrides())
            assert ret['model'] == frame.get('model', frame_name), frame_name
            assert ret['waf_target'] == frame.get('waf_target', vinfo.default_waf_target(name)), frame_name
            assert isinstance(ret['sitl-port'], bool), frame_name
            for key in 'default_params_filename', 'periph_params_filename':
                assert ret.get(key) == frame.get(key), frame_name


def test_options_for_frame_overrides():
    vinfo = vehicleinfo.VehicleInfo()
    ret = vinfo.options_for_frame('quad', 'ArduCopter', Namespace(model='mymodel', build_target='bin/mytarget'))
    assert ret['model'] == 'mymodel'
    assert ret['waf_target'] == 'bin/mytarget'
    # xplane and flightaxis models have no SITL port
    ret = vinfo.options_for_frame('quad', 'ArduCopter', Namespace(model='xplane', build_target=None))
    assert ret['sitl-port'] is False


def test_options_for_frame_falls_back_by_prefix(info):
    vinfo = vehicleinfo.VehicleInfo()
    ret = vinfo.options_for_frame('octa-foo', 'ArduCopter', no_overrides())
    octa = info['ArduCopter']['frames']['octa']
    assert ret['default_params_filename'] == octa['default_params_filename']
    # the model is the frame asked for, not the one it fell back to
    assert ret['model'] == 'octa-foo'


def test_options_for_frame_does_not_change_the_loaded_options(info):
    vinfo = vehicleinfo.VehicleInfo()
    vinfo.options_for_frame('octa-foo', 'ArduCopter', no_overrides())
    assert vinfo.options == info
    assert vinfo.options_for_frame('octa', 'ArduCopter', no_overrides())['model'] == 'octa'

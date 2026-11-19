# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for singleton constants used by AP_Scripting examples."""

import copy
import os
import shutil
import subprocess

import bindings
import example_constants_binding
import pytest


@pytest.fixture(scope='module')
def ir():
    return bindings.read_ir()


def _copy_sources(tmp_path):
    root = example_constants_binding.ROOT
    paths = [
        'libraries/AP_Scripting/generator/description/bindings.desc',
        'libraries/AP_GPS/AP_GPS.h',
        'libraries/AP_GPS/AP_GPS_FixType.h',
        'libraries/AP_Terrain/AP_Terrain.h',
        'libraries/AP_Mission/AP_Mission.h',
    ]
    for relative in paths:
        destination = tmp_path / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root / relative, destination)


def _replace(path, old, new):
    text = path.read_text()
    assert text.count(old) == 1
    path.write_text(text.replace(old, new))


def test_selection_validates_sources_and_emits_constants(ir, tmp_path):
    spec = example_constants_binding.select(ir)
    assert len(spec['constants']) == 13
    assert spec['constants'][0] == dict(name='GPS_NO_GPS', value=0)
    assert spec['constants'][6] == dict(name='GPS_OK_FIX_3D_RTK_FIXED', value=6)
    assert spec['constants'][8] == dict(name='TERRAIN_STATUS_UNHEALTHY', value=1)
    assert spec['constants'][11] == dict(name='MISSION_RUNNING', value=1)
    output = example_constants_binding.rust(spec, '// generated')
    assert output.startswith('// generated\n\n')
    assert 'pub const GPS_OK_FIX_3D: u8 = 3;' in output
    assert 'pub const TERRAIN_STATUS_UNHEALTHY: u8 = 1;' in output
    assert 'pub const MISSION_COMPLETE: u8 = 2;' in output
    source = tmp_path / 'constants.rs'
    source.write_text('#![no_std]\n' + output)
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--crate-type', 'lib', '--edition', '2021',
                    str(source), '-o', str(tmp_path / 'libconstants.rlib')], check=True)


@pytest.mark.parametrize('owner,field,value', [
    ('AP_GPS', 'rename', 'position'),
    ('AP_Terrain', 'depends', '1'),
    ('AP_Mission', 'scheduler_semaphore', False),
])
def test_selection_rejects_owner_metadata_drift(ir, owner, field, value):
    changed = copy.deepcopy(ir)
    next(item for item in changed['singletons'] if item['name'] == owner)[field] = value
    with pytest.raises(ValueError, match=f'unaudited {owner} owner metadata'):
        example_constants_binding.select(changed)


def test_selection_rejects_descriptor_drift(ir, tmp_path):
    _copy_sources(tmp_path)
    descriptor = tmp_path / 'libraries/AP_Scripting/generator/description/bindings.desc'
    _replace(descriptor, 'GPS_OK_FIX_3D GPS_OK_FIX_3D_DGPS', 'GPS_OK_FIX_3D_DGPS GPS_OK_FIX_3D')
    with pytest.raises(ValueError, match='unaudited AP_GPS enum constants'):
        example_constants_binding.select(ir, tmp_path)


def test_selection_rejects_gps_alias_or_value_drift(ir, tmp_path):
    _copy_sources(tmp_path)
    gps = tmp_path / 'libraries/AP_GPS/AP_GPS.h'
    _replace(gps, 'AP_GPS_FixType::RTK_FLOAT;', 'AP_GPS_FixType::RTK_FIXED;')
    with pytest.raises(ValueError, match='unaudited AP_GPS constant GPS_OK_FIX_3D_RTK_FLOAT'):
        example_constants_binding.select(ir, tmp_path)
    _copy_sources(tmp_path)
    fix_type = tmp_path / 'libraries/AP_GPS/AP_GPS_FixType.h'
    _replace(fix_type, 'FIX_3D = 3,', 'FIX_3D = 9,')
    with pytest.raises(ValueError, match='unaudited AP_GPS constant GPS_OK_FIX_3D'):
        example_constants_binding.select(ir, tmp_path)


@pytest.mark.parametrize('relative,old,new,message', [
    ('libraries/AP_Terrain/AP_Terrain.h', 'TerrainStatusUnhealthy = 1', 'TerrainStatusUnhealthy = 9',
     'unaudited AP_Terrain constant TerrainStatusUnhealthy'),
    ('libraries/AP_Mission/AP_Mission.h', 'MISSION_RUNNING=1', 'MISSION_RUNNING=9',
     'unaudited AP_Mission constant MISSION_RUNNING'),
])
def test_selection_rejects_direct_enum_value_drift(ir, tmp_path, relative, old, new, message):
    _copy_sources(tmp_path)
    _replace(tmp_path / relative, old, new)
    with pytest.raises(ValueError, match=message):
        example_constants_binding.select(ir, tmp_path)


def test_emission_rejects_mutated_specification():
    spec = example_constants_binding.specification()
    spec['constants'][0]['value'] = 8
    with pytest.raises(ValueError, match='unaudited example singleton constants specification'):
        example_constants_binding.rust(spec, '')

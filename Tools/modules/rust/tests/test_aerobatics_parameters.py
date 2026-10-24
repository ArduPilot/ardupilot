# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compare real applet parameters, including persistence when changing engines."""

import re
import shutil
import struct
import time

from pymavlink import mavutil
from test_aerobatics_path import APPLET
from test_aerobatics_path import LUA_DIR
from test_runtime import Vehicle

import build


def table(vehicle):
    vehicle.link.mav.param_request_list_send(1, 1)
    rows = {}
    seen = set()
    previous = None
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        m = vehicle.receive()
        if m is None or m.get_type() != 'PARAM_VALUE':
            continue
        seen.add(m.param_index)
        if m.param_id.startswith(('AEROM_', 'TRIK')):
            rows[m.param_id] = (m.param_index, m.param_value)
        if len(seen) == m.param_count:
            # Dynamic registration can change the list during a stream. Do not
            # compare persistence against a snapshot missing its final row.
            snapshot = (m.param_count, rows)
            if snapshot == previous:
                return rows
            previous = snapshot
            rows = {}
            seen = set()
            vehicle.link.mav.param_request_list_send(1, 1)
    raise AssertionError(f'Incomplete parameter list: {len(seen)} entries; {vehicle.diagnostic()}')


def restart(vehicle, engine, counter):
    mark = len(vehicle.texts)
    if engine == 'rust':
        vehicle.restart(counter)
        vehicle.text('Enabled', after=mark)
    else:
        vehicle.link.mav.command_int_send(1, 1, 0, mavutil.mavlink.MAV_CMD_SCRIPTING, 0, 0,
                                          mavutil.mavlink.SCRIPTING_CMD_STOP_AND_RESTART, 0, 0, 0, 0, 0, 0)
        vehicle.text('Enabled', after=mark)


def test_airshow_parameters(tmp_path):
    lua = (LUA_DIR / 'plane_aerobatics.lua').read_text()
    declarations = re.findall(r"bind_add_param\('([^']+)',\s*(\d+),\s*(-?[\d.]+)\)", lua)
    rust = (APPLET / 'parameters.rs').read_text().split('pub struct Parameters')[0]
    native = re.findall(r'\((\d+), "([^"]+)", (-?[\d.]+)\)', rust)
    assert ('35', 'LANG', '0.0') in native
    assert {(name, int(index), float(default)) for name, index, default in declarations} == \
        {(name, int(index), float(default)) for index, name, default in native if name != 'LANG'}
    assert len(declarations) == 29
    module = build.build(APPLET / 'main.rs', tmp_path / 'airshow.apmod', 'x86_64-unknown-linux-gnu',
                         16384, heap_size=32768)
    results = {}
    for engine in ('rust', 'lua'):
        vehicle = Vehicle(tmp_path / engine, {'airshow.apmod': module.read_bytes()}, {
            'MOD_ENABLE': int(engine == 'rust'), 'SCR_ENABLE': int(engine == 'lua'),
            'MOD_HEAP_MAX': 32768, 'MOD_CODE_MAXSIZE': 131072, 'MOD_STACK_MAX': 32768,
            'SCR_HEAP_SIZE': 350000, 'SCR_VM_I_COUNT': 1000000,
        })
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        shutil.copyfile(LUA_DIR / 'plane_aerobatics.lua', scripts / 'plane_aerobatics.lua')
        try:
            vehicle.start()
            vehicle.text('Loaded plane_aerobatics')
            initial = table(vehicle)
            selector = {'AEROM_LANG'} if engine == 'rust' else set()
            assert set(initial) == {'AEROM_' + row[0] for row in declarations} | {'TRIK_ENABLE'} | selector
            if engine == 'rust':
                assert initial['AEROM_LANG'][1] == 0
            for name, _, default in declarations:
                assert struct.pack('<f', initial['AEROM_' + name][1]) == struct.pack('<f', float(default))
            assert initial['TRIK_ENABLE'][1] == 0
            vehicle.parameter('TRIK_ENABLE', 1)
            restart(vehicle, engine, 1)
            assert vehicle.parameter('TRIK_COUNT') == 3
            vehicle.parameter('TRIK_COUNT', 11)
            restart(vehicle, engine, 2)
            results[engine] = table(vehicle)
            expected = {'AEROM_' + row[0] for row in declarations} | {
                'TRIK_ENABLE', 'TRIK_SEL_FN', 'TRIK_ACT_FN', 'TRIK_COUNT'}
            expected |= {f'TRIK{i}_{suffix}' for i in range(1, 12) for suffix in ('ID', 'ARG1', 'ARG2', 'ARG3', 'ARG4')}
            assert set(results[engine]) == expected | selector
            vehicle.parameter('AEROM_ANG_ACCEL', 50)
            vehicle.parameter('AEROM_ANG_TC', 2)
            vehicle.parameter('TRIK1_ID', 72)
            restart(vehicle, engine, 3)
            repaired = table(vehicle)
            assert repaired['AEROM_ANG_ACCEL'][1] == 3000
            assert struct.pack('<f', repaired['AEROM_ANG_TC'][1]) == struct.pack('<f', 0.2)
            # Reboot into the other engine with exactly the same backing store.
            vehicle.parameter('MOD_ENABLE', int(engine == 'lua'))
            vehicle.parameter('SCR_ENABLE', int(engine == 'rust'))
            vehicle.pump(0.5)
            vehicle.stop()
            mark = len(vehicle.texts)
            vehicle.start()
            vehicle.text('Loaded plane_aerobatics', after=mark)
            persisted = table(vehicle)
            assert {k: v[1] for k, v in persisted.items() if k != 'AEROM_LANG'} == \
                {k: v[1] for k, v in repaired.items() if k != 'AEROM_LANG'}
        finally:
            vehicle.stop()
    # Absolute MAVLink enumeration indices can change with enabled core groups;
    # storage indices above and the table-relative order must remain identical.
    common_rust = {k: v for k, v in results['rust'].items() if k != 'AEROM_LANG'}
    assert {k: v[1] for k, v in common_rust.items()} == {k: v[1] for k, v in results['lua'].items()}
    assert sorted(common_rust, key=lambda k: common_rust[k][0]) == \
        sorted(results['lua'], key=lambda k: results['lua'][k][0])

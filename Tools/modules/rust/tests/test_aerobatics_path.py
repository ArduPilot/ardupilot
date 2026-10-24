# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compare the complete native AirShow trajectory with the original Lua schedule."""

import math
import os
import re
import shutil
import struct
import subprocess

import pytest

from pymavlink import DFReader
from rust.build import sdk_environment
from test_math_runtime import chibios_probe
from test_runtime import ROOT
from test_runtime import Vehicle
from test_threads import renode  # noqa: F401

import build

APPLET = ROOT / 'libraries/AP_Module/applets/aerobatics'
LUA_DIR = ROOT / 'libraries/AP_Scripting/applets/Aerobatics/FixedWing'


def lua_airshow_schedule():
    """Use the same ten-maneuver prefix for Rust/Lua comparisons."""
    lines = (LUA_DIR / 'Schedules/AirShow.txt').read_text().splitlines(keepends=True)
    end = next(i for i, line in enumerate(lines) if line.startswith('barrel_roll '))
    return ''.join(lines[:end + 1])


def test_airshow_schedule_matches_lua():
    def sequence(text):
        lines = text.splitlines()
        start = next(i for i, line in enumerate(lines) if line.startswith('message:'))
        return [line.split() for line in lines[start:] if line.strip() and not line.lstrip().startswith('#')]

    native = sequence((APPLET / 'AirShow.txt').read_text())
    assert sum(line[0] == 'message:' for line in native) == 10
    assert native == sequence(lua_airshow_schedule())


def test_builtin_ids_match_lua():
    lua = (LUA_DIR / 'plane_aerobatics.lua').read_text()
    rust = (APPLET / 'maneuvers.rs').read_text()
    commands = dict((int(i), (function, name)) for i, function, name in re.findall(
        r'command_table\[(\d+)\]\s*=\s*PathFunction\((\w+), "([^"]+)"\)', lua))
    native = dict((int(i), (variant, name)) for i, variant, name in re.findall(
        r'(\d+) => \(Self::(\w+), "([^"]+)"\)', rust))
    named = dict(re.findall(r'"([^"]+)" => Self::(\w+)', rust))
    assert len(native) == 10
    assert all((named[commands[i][0]], commands[i][1]) == binding for i, binding in native.items())


def test_parser_units(tmp_path):
    rustc = os.environ.get('RUSTC', 'rustc')
    environment = sdk_environment()
    sdk = tmp_path / 'libap_rust_sdk.rlib'
    subprocess.run([rustc, '--edition=2021', '--crate-name', 'ap_rust_sdk', '--crate-type', 'rlib',
                    str(ROOT / 'libraries/AP_Module/sdk/rust/src/lib.rs'), '-o', str(sdk)], check=True, env=environment)
    source = tmp_path / 'test.rs'
    source.write_text('extern crate alloc;\n' + '\n'.join(
        f'#[path="{APPLET / (name + ".rs")}"] mod {name};'
        for name in ('math', 'primitives', 'path', 'maneuvers', 'schedule', 'support', 'mission')))
    executable = tmp_path / 'tests'
    subprocess.run([rustc, '--edition=2021', '--test', '-A', 'dead_code', str(source),
                    '--extern', f'ap_rust_sdk={sdk}', '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


@pytest.fixture(scope='module')
def composition_results(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp('airshow-composition')
    if os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Golden oracle requires Lua')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/aerobatics_path.rs',
                        tmp_path / 'path.apmod', 'x86_64-unknown-linux-gnu', 16384, heap_size=32768)
    vehicle = Vehicle(tmp_path / 'vehicle', {'path.apmod': image.read_bytes()}, {
        'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 500000, 'SCR_VM_I_COUNT': 1000000,
        'MOD_HEAP_MAX': 65536, 'MOD_CODE_MAXSIZE': 131072,
        'LOG_BACKEND_TYPE': 1, 'LOG_DISARMED': 1, 'LOG_FILE_DSRMROT': 0, 'LOG_FILE_BUFSIZE': 128,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    shutil.copyfile(APPLET / 'AirShow.txt', vehicle.directory / 'trick72.txt')
    (scripts / 'trick72.txt').write_text(lua_airshow_schedule())
    source = (LUA_DIR / 'plane_aerobatics.lua').read_text()
    marker = '\nfunction update()\n'
    assert source.count(marker) == 1
    source = source.split(marker)[0] + '''
local current_speed = 18.0
function target_groundspeed() return current_speed end
local original_gcs = gcs
gcs = {send_text=function(self, severity, text) original_gcs:send_text(severity, "L:"..text) end}
local function golden()
 load_trick(72)
 local path = command_table[72].fn
 local len = path:get_length()
 local q = path:get_final_orientation()
 for i=0,500 do
  local t = i / 500.0
  if i > 0 and i % 7 == 0 and i < 475 then t = t + 0.05 end
  current_speed = 18.0 + (i % 11)
  AEROM_ENTRY_RATE:set(60.0 + (i % 9))
  local p = path:get_pos(t)
  logger.write("ALCP","Id,T,X,Y,Z,Roll,Cor,Len,Q1,Q2,Q3,Q4,Next,Boost","HffffffffffffB",
   i,t,p:x(),p:y(),p:z(),path:get_roll(t,len/18.0),path:get_roll_correction(t),len,
   q:q1(),q:q2(),q:q3(),q:q4(),path:get_next_segment_start(t),path:get_attribute(t,"thr_boost") and 1 or 0)
 end
 gcs:send_text(6,"AEROP Lua composition done")
end
return golden,2000
'''
    (scripts / 'golden.lua').write_text(source)
    try:
        vehicle.start()
        vehicle.text('AEROP Lua composition done', timeout=30)
        vehicle.text('Module: AEROP composition done', timeout=30)
        vehicle.pump(2)
        assert not any('panic' in line or 'returned -' in line for line in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()
    names = [line.split(':', 1)[1].strip() for line in (APPLET / 'AirShow.txt').read_text().splitlines()
             if line.startswith('message:')]
    rust_messages = [text for text in vehicle.texts if text in names]
    lua_messages = [text[2:] for text in vehicle.texts if text.startswith('L:') and text[2:] in names]
    assert rust_messages == lua_messages == names, (rust_messages, lua_messages)
    rows = {'ARCP': {}, 'ALCP': {}}
    for file in sorted((vehicle.directory / 'logs').glob('*.BIN')):
        reader = DFReader.DFReader_binary(str(file))
        while (message := reader.recv_msg()) is not None:
            if message.get_type() in rows:
                rows[message.get_type()][message.Id] = message.to_dict()
    assert len(rows['ARCP']) == len(rows['ALCP']) == 501
    maximum = {}
    for i, actual in rows['ARCP'].items():
        for field, value in actual.items():
            if field in ('mavpackettype', 'TimeUS'):
                continue
            expected = rows['ALCP'][i][field]
            assert math.isfinite(value) and math.isfinite(expected), (i, field, value, expected)
            maximum[field] = max(maximum.get(field, 0), abs(value - expected))
            assert struct.pack('<f', value) == struct.pack('<f', expected), (i, field, value, expected)
    print('AirShow path maximum absolute differences:', maximum)
    memory = [tuple(map(int, re.search(r'Module RAM: heap \d+ used (\d+) peak (\d+)', text).groups()))
              for text in vehicle.texts if re.search(r'Module RAM: heap \d+ used (\d+) peak (\d+)', text)]
    assert memory[-1][0] == 0 and 0 < memory[-1][1] < 32768, memory
    print('AirShow SITL final heap used/peak:', memory[-1])
    oracle = bytearray()
    for i in sorted(rows['ALCP']):
        for field, value in rows['ALCP'][i].items():
            if field not in ('mavpackettype', 'TimeUS'):
                oracle.extend(struct.pack('<f', value))
    return {'oracle': oracle, 'peak': memory[-1][1]}


def test_airshow_path(composition_results):
    assert composition_results


def test_airshow_h743(tmp_path, renode, composition_results):  # noqa: F811
    chibios_probe(tmp_path, renode, 'aerobatics_path', composition_results['oracle'],
                  {'trick72.txt': (APPLET / 'AirShow.txt').read_bytes()}, heap_limit=28672,
                  expected_heap=65536, expected_code=131072)


@pytest.mark.parametrize('heap_size', [1024, 8192, 16384])
def test_allocation_failure_reload(tmp_path, heap_size):
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/aerobatics_path.rs',
                        tmp_path / 'path.apmod', 'x86_64-unknown-linux-gnu', 16384, heap_size=heap_size)
    vehicle = Vehicle(tmp_path / 'vehicle', {'path.apmod': image.read_bytes()},
                      {'MOD_HEAP_MAX': heap_size, 'MOD_CODE_MAXSIZE': 131072})
    shutil.copyfile(APPLET / 'AirShow.txt', vehicle.directory / 'trick72.txt')
    try:
        vehicle.start()
        for cycle in range(3):
            mark = len(vehicle.texts) if cycle else 0
            if cycle:
                vehicle.restart(cycle)
            vehicle.text('path.apmod returned -2', after=mark)
            vehicle.text('Module: exited path.apmod', after=mark)
            vehicle.pump(0.5)
            memory = [text for text in vehicle.texts[mark:] if 'Module RAM: heap ' in text]
            assert ' used 0 ' in memory[-1], memory
            assert not any('panic' in text for text in vehicle.texts)
        if heap_size >= 8192:
            # Limits are boot-time parameters. Prove the failed allocation
            # doesn't poison later generations, using a path that fits them.
            (vehicle.directory / 'trick72.txt').write_text('name: Short\nloop 25\n')
            vehicle.restart(4)
            vehicle.text('Module: AEROP composition done', timeout=30)
    finally:
        vehicle.stop()


def test_sampling_has_no_allocations(tmp_path, monkeypatch, composition_results):
    monkeypatch.setenv('AP_MODULE_AERO_BUILD_ONLY', '1')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/aerobatics_path.rs',
                        tmp_path / 'path.apmod', 'x86_64-unknown-linux-gnu', 16384, heap_size=32768)
    vehicle = Vehicle(tmp_path / 'vehicle', {'path.apmod': image.read_bytes()},
                      {'MOD_HEAP_MAX': 65536, 'MOD_CODE_MAXSIZE': 131072})
    shutil.copyfile(APPLET / 'AirShow.txt', vehicle.directory / 'trick72.txt')
    try:
        vehicle.start()
        vehicle.text('Module: AEROP build done')
        vehicle.text('Module: exited path.apmod')
        vehicle.pump(0.5)
        memory = [tuple(map(int, re.search(r'heap \d+ used (\d+) peak (\d+)', text).groups()))
                  for text in vehicle.texts if re.search(r'heap \d+ used (\d+) peak (\d+)', text)]
        assert memory[-1] == (0, composition_results['peak']), memory
    finally:
        vehicle.stop()

# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Golden comparisons against the original Lua aerobatics implementation."""

import math
import os
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

LUA = ROOT / 'libraries/AP_Scripting/applets/Aerobatics/FixedWing/plane_aerobatics.lua'


def test_host_units(tmp_path):
    rustc = os.environ.get('RUSTC', 'rustc')
    environment = sdk_environment()
    sdk = tmp_path / 'libap_rust_sdk.rlib'
    subprocess.run([rustc, '--edition=2021', '--crate-name', 'ap_rust_sdk', '--crate-type', 'rlib',
                    str(ROOT / 'libraries/AP_Module/sdk/rust/src/lib.rs'), '-o', str(sdk)], check=True, env=environment)
    executable = tmp_path / 'aerobatics-tests'
    subprocess.run([rustc, '--edition=2021', '--test', '-A', 'dead_code',
                    str(ROOT / 'libraries/AP_Module/applets/aerobatics/tests.rs'),
                    '--extern', f'ap_rust_sdk={sdk}', '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


@pytest.fixture(scope='module')
def primitive_results(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp('aerobatics-primitives')
    if os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Golden oracle requires Lua')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/aerobatics_primitives.rs',
                        tmp_path / 'primitives.apmod', 'x86_64-unknown-linux-gnu', 8192, heap_size=32768)
    vehicle = Vehicle(tmp_path / 'vehicle', {'primitives.apmod': image.read_bytes()}, {
        'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 300000, 'SCR_VM_I_COUNT': 1000000,
        'LOG_BACKEND_TYPE': 1, 'LOG_DISARMED': 1, 'LOG_FILE_DSRMROT': 0, 'LOG_FILE_BUFSIZE': 128,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    # Run the actual upstream functions, not a second implementation in Python.
    source = LUA.read_text()
    marker = 'local _path_composer = inheritsFrom(nil)'
    assert source.count(marker) == 1
    source = source.split(marker)[0]
    source += '''
local paths = {
 path_straight(80), path_straight(-2.5), path_reverse(60),
 path_vertical_arc(25,360), path_vertical_arc(-40,225), path_vertical_arc(20.5,-540), path_vertical_arc(0,90),
 path_horizontal_arc(-75,360,0), path_horizontal_arc(30.5,225,20), path_horizontal_arc(-30.5,-270,-15),
 path_cylinder(30,100,2), path_cylinder(-30,100,2), path_cylinder(25,60,0.5),
}
local times = {-0.1,0,0.02,0.1,0.25,0.4999,0.5,0.5001,0.75,0.9,1,1.1}
local function golden()
 for id,path in ipairs(paths) do
  local len = path:get_length()
  local q = path:get_final_orientation()
  local ext = path:get_extents_x()
  for i,t in ipairs(times) do
   local p = path:get_pos(t)
   logger.write("ALPG", "Id,I,T,X,Y,Z,Len,Q1,Q2,Q3,Q4,Cor,Lo,Hi", "BBffffffffffff",
    id-1,i-1,t,p:x(),p:y(),p:z(),len,q:q1(),q:q2(),q:q3(),q:q4(),path:get_roll_correction(t),ext[1],ext[2])
  end
 end
 local rolls = {roll_angle(720),roll_angle(0),roll_angle_entry_exit(90),roll_angle_entry_exit(-90),
  roll_angle_entry_exit(0),roll_angle_entry(90),roll_angle_entry(0),roll_angle_exit(-90),roll_angle_exit(0),
  roll_sequence({{1,0},{2,180},{1,0},{3,-360}})}
 for id,roll in ipairs(rolls) do
  for j,duration in ipairs({0.5,2,10}) do
   for i,t in ipairs(times) do
    local angle = roll:get_roll(t,duration)
    logger.write("ALLR", "Id,J,I,Ang,Wrap", "BBBff",id-1,j-1,i-1,angle,wrap_180(angle))
   end
  end
 end
 for id,yaw in ipairs({0,90,90.1,180,-90.1}) do
  for j=1,3 do
   local path
   if j==1 then path=path_align_center() else path=path_align_box(j==2 and 1 or -1) end
   path:set_next_extents({-30,40},makeVector3f(15,0,0),qorient(0,0,yaw))
   logger.write("ALAL","Id,J,D","BBf",id-1,j-1,path:get_length())
  end
 end
 for id,a in ipairs({-540.0,-360.0,-180.0,-0.0,0.0,180.0,360.0,540.0,180.00002,1000000.0,0/0,1/0,-1/0}) do
  logger.write("ALWM","Id,In,D,R,W360,W180,Wpi,W2pi,Sign,Min,Max,Con,Alpha","Bffffffffffff",
   id-1,a,math.deg(a),math.rad(a),wrap_360(a),wrap_180(a),wrap_pi(a),wrap_2pi(a),sgn(a),
   math.min(a,0.0),math.max(a,0.0),constrain(a,-90.0,90.0),calc_lowpass_alpha(0.025,0.1))
 end
 for id,a in ipairs({{0,0,0},{30,45,-90},{-170,-80,179}}) do
  local q = qorient(a[1],a[2],a[3])
  local v = makeVector3f(1.2,-2.3,3.4)
  local r = quat_earth_to_body(q,v)
  local b = quat_body_to_earth(q,v)
  logger.write("ALQM","Id,Q1,Q2,Q3,Q4,RX,RY,RZ,BX,BY,BZ,DCM","Bfffffffffff",
   id-1,q:q1(),q:q2(),q:q3(),q:q4(),r:x(),r:y(),r:z(),b:x(),b:y(),b:z(),get_quat_dcm_c_y(q))
 end
 for id,b in ipairs({{1,0,0},{0,1,0},{-1,0.01,0},{1,2,3}}) do
  local a = makeVector3f(1,0,0)
  local v = makeVector3f(b[1],b[2],b[3])
  local q = vectors_to_quat_rotation(a,v)
  local r = tangents_to_rate(a,v,0.025)
  logger.write("ALVM","Id,Q1,Q2,Q3,Q4,X,Y,Z","Bfffffff",id-1,q:q1(),q:q2(),q:q3(),q:q4(),r:x(),r:y(),r:z())
 end
 gcs:send_text(6,"AEROP Lua primitives done")
end
return golden, 2000
'''
    (scripts / 'golden.lua').write_text(source)
    try:
        vehicle.start()
        vehicle.text('AEROP Lua primitives done', timeout=30)
        vehicle.text('Module: AEROP primitives done', timeout=30)
        vehicle.pump(2)
        assert not any('panic' in text or 'returned -' in text for text in vehicle.texts)
    finally:
        vehicle.stop()
    rows = {}
    for file in sorted((vehicle.directory / 'logs').glob('*.BIN')):
        reader = DFReader.DFReader_binary(str(file))
        while (message := reader.recv_msg()) is not None:
            kind = message.get_type()
            if kind in ('ARPG', 'ALPG', 'ARRL', 'ALLR', 'ARAL', 'ALAL', 'ARWM', 'ALWM', 'ARQM', 'ALQM', 'ARVM', 'ALVM'):
                row = message.to_dict()
                key = tuple(row.get(field) for field in ('Id', 'J', 'I'))
                rows.setdefault(kind, {})[key] = row
    oracle = bytearray()
    for native, lua, count in [('ARPG', 'ALPG', 156), ('ARRL', 'ALLR', 360), ('ARAL', 'ALAL', 15),
                               ('ARWM', 'ALWM', 13), ('ARQM', 'ALQM', 3), ('ARVM', 'ALVM', 4)]:
        assert len(rows.get(native, {})) == count
        assert len(rows.get(lua, {})) == count
        assert rows[native].keys() == rows[lua].keys()
        for key, actual in rows[native].items():
            expected = rows[lua][key]
            for field, value in actual.items():
                if field in ('mavpackettype', 'TimeUS'):
                    continue
                reference = expected[field]
                if isinstance(value, float) and math.isnan(reference):
                    assert math.isnan(value), (native, key, field, value, reference)
                else:
                    # f32 throughout the native applet versus Lua's mixed
                    # integer/float intermediates: only these signed-zero
                    # differences are allowed on the same host/libm.
                    zero_policy = native == 'ARPG' and (
                        (key[0] in (6, 7) and field == 'Z') or
                        (key == (8, None, 1) and field in ('Z', 'Cor')))
                    if zero_policy and value == reference == 0:
                        continue
                    assert struct.pack('<f', value) == struct.pack('<f', reference), (
                        native, key, field, value, reference)
        for key in sorted(rows[lua]):
            for field, value in rows[lua][key].items():
                if field not in ('mavpackettype', 'TimeUS'):
                    oracle.extend(struct.pack('<f', value))
    return oracle


def test_primitives(primitive_results):
    assert primitive_results


def test_primitives_h743(tmp_path, renode, primitive_results):  # noqa: F811
    chibios_probe(tmp_path, renode, 'aerobatics_primitives', primitive_results)

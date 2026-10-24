# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Feed identical synthetic observations to the original Lua and native controller."""

import math
import os
import shutil
import struct

import pytest

from pymavlink import DFReader
from test_aerobatics_path import APPLET
from test_aerobatics_path import LUA_DIR
from test_aerobatics_path import lua_airshow_schedule
from test_math_runtime import chibios_probe
from test_runtime import ROOT
from test_runtime import Vehicle
from test_threads import renode  # noqa: F401

import build


def lua_trace(variant):
    source = (LUA_DIR / 'plane_aerobatics.lua').read_text()
    marker = '\nfunction update()\n'
    assert source.count(marker) == 1
    source = source.split(marker)[0] + '''
local observation = 0
local now_ms = 10000
local pose = makeVector3f(0,0,-29)
local attitude = Quaternion()
local gyro = makeVector3f(0.1,0.2,0.3)
local velocity = makeVector3f(18,0.02,-0.01)
local airspeed = 18.0
local eas2tas = 1.0
local variant = "VARIANT"
if variant == "mirror" then
 AEROM_PATH_SCALE:set(-1.2)
 AEROM_LKAHD:set(0.0)
 AEROM_KE_RUDD:set(0.0)
 AEROM_ANG_ACCEL:set(0.0)
 AEROM_YAW_ACCEL:set(500.0)
 assert(param:set("AEROM_ERR_COR_P",1.5))
end
local location = Location()
local records = {}
local outputs = {}
local rudder = 0
local real_logger = logger
logger = {write=function(name,labels,format,...) records[name] = {...} end}
millis = function() return {tofloat=function() return now_ms*1.0 end} end
ahrs = {
 get_relative_position_NED_origin=function() return pose end,
 get_position=function() return location end, get_origin=function() return location end,
 get_gyro=function() return gyro end, get_velocity_NED=function() return velocity end,
 airspeed_EAS=function() return airspeed end, get_EAS2TAS=function() return eas2tas end,
 get_quaternion=function() return attitude end,
}
SRV_Channels = {get_output_scaled=function() return 40.0 end}
vehicle = {
 set_target_throttle_rate_rpy=function(self,thr,r,p,y) outputs={thr,r,p,y} end,
 set_rudder_offset=function(self,r) rudder=r end,
}
local function inputs(i)
 now_ms = 10000 + i*25
 pose = makeVector3f(i*0.45,math.sin(i*0.01)*2.0,-30.0+math.cos(i*0.02))
 attitude = Quaternion()
 attitude:from_euler(math.sin(i*0.03)*0.2,math.cos(i*0.01)*0.1,0.3)
 if variant ~= "nominal" then
  velocity:x(i%37==0 and -5.0 or 10.0+(i%21))
  airspeed = 4.0 + (i%37)
  eas2tas = i%43==0 and 5.0 or 1.0
  gyro:x(math.sin(i*0.02))
 end
end
local function golden()
 if observation==0 then
  inputs(0)
  load_trick(72)
  current_task = PathTask(command_table[72].fn,"Replay",nil,0)
  if variant == "mirror" then path_var.count = 200 end
  assert(do_path())
 end
 for _=1,4 do
  observation = observation+1
  inputs(observation)
  if variant ~= "nominal" then path_var.path_t = observation / 520.0 end
  assert(do_path())
  local rt,tm,sp = records.AERT,records.AETM,records.AESP
  real_logger.write("ALCT","Id,Thr,R,P,Y,Rudd,Perr,Pt,Dt,Ert,Qct","Hffffffffff",
   observation,outputs[1],outputs[2],outputs[3],outputs[4],rudder,rt[11],tm[1],tm[4],tm[2],tm[3])
  local post = records.POST
  real_logger.write("ALCS","Id,X,Y,Z,Q1,Q2,Q3,Q4,Cx,Cy,Cz,Px,Py,Pz,Ex","Hffffffffffffff",
   observation,post[1],post[2],post[3],post[4],post[5],post[6],post[7],rt[1],rt[2],rt[3],rt[4],rt[5],rt[6],rt[7])
  real_logger.write("LASP","Id,Targ,Curr,P,I,Total,Add","Hffffff",observation,sp[1],sp[2],sp[3],sp[4],sp[5],sp[6])
 end
 if observation < 512 then return golden,1 end
 gcs:send_text(6,"AEROP Lua controller done")
end
return golden,2000
'''.replace('VARIANT', variant)
    return source


@pytest.fixture(scope='module', params=['nominal', 'sweep', 'mirror'])
def controller_results(tmp_path_factory, request):
    tmp_path = tmp_path_factory.mktemp('controller-trace')
    if os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Golden oracle requires Lua')
    with pytest.MonkeyPatch.context() as patch:
        patch.delenv('AP_MODULE_AERO_CONTROLLER_VARIANT', raising=False)
        if request.param != 'nominal':
            patch.setenv('AP_MODULE_AERO_CONTROLLER_VARIANT', request.param)
        image = build.build(ROOT / 'libraries/AP_Module/tests/modules/aerobatics_controller.rs',
                            tmp_path / 'controller.apmod', 'x86_64-unknown-linux-gnu', 16384, heap_size=32768)
    vehicle = Vehicle(tmp_path / 'vehicle', {'controller.apmod': image.read_bytes()}, {
        'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 500000, 'SCR_VM_I_COUNT': 1000000,
        'MOD_HEAP_MAX': 65536, 'MOD_CODE_MAXSIZE': 131072,
        'LOG_BACKEND_TYPE': 1, 'LOG_DISARMED': 1, 'LOG_FILE_DSRMROT': 0, 'LOG_FILE_BUFSIZE': 128,
        'AIRSPEED_MIN': 9, 'AIRSPEED_CRUISE': 18, 'SCALING_SPEED': 15,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    shutil.copyfile(APPLET / 'AirShow.txt', vehicle.directory / 'trick72.txt')
    (scripts / 'trick72.txt').write_text(lua_airshow_schedule())
    (scripts / 'golden.lua').write_text(lua_trace(request.param))
    try:
        vehicle.start()
        vehicle.text('AEROP Lua controller done', timeout=30)
        vehicle.text('Module: AEROP controller done', timeout=30)
        vehicle.pump(2)
        assert not any('panic' in line or 'returned -' in line for line in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()
    rows = {name: {} for name in ('ARCT', 'ALCT', 'ARCS', 'ALCS', 'RASP', 'LASP')}
    for file in sorted((vehicle.directory / 'logs').glob('*.BIN')):
        reader = DFReader.DFReader_binary(str(file))
        while (message := reader.recv_msg()) is not None:
            if message.get_type() in rows:
                rows[message.get_type()][message.Id] = message.to_dict()
    for native, lua in [('ARCT', 'ALCT'), ('ARCS', 'ALCS'), ('RASP', 'LASP')]:
        assert len(rows[native]) == len(rows[lua]) == 512
        maximum = {}
        for i, row in rows[native].items():
            for field, actual in row.items():
                if field in ('mavpackettype', 'TimeUS'):
                    continue
                expected = rows[lua][i][field]
                assert math.isfinite(actual) and math.isfinite(expected), (native, i, field, actual, expected)
                maximum[field] = max(maximum.get(field, 0), abs(actual-expected))
                assert struct.pack('<f', actual) == struct.pack('<f', expected), (native, i, field, actual, expected)
        print(native, 'maximum absolute differences', maximum)
    oracle = bytearray()
    for i in sorted(rows['ALCT']):
        for kind in ('ALCT', 'ALCS', 'LASP'):
            for field, value in rows[kind][i].items():
                if field not in ('mavpackettype', 'TimeUS'):
                    oracle.extend(struct.pack('<f', value))
    return request.param, oracle


def test_controller_trace(controller_results):
    assert controller_results


def test_controller_h743(tmp_path, renode, controller_results, monkeypatch):  # noqa: F811
    from aerobatics_board_oracle import generate
    variant, _host_oracle = controller_results
    # Compare with Lua on the same target, not a Linux-libm reference.
    oracle = generate(tmp_path / 'lua', renode, variant)
    monkeypatch.delenv('AP_MODULE_AERO_CONTROLLER_VARIANT', raising=False)
    if variant != 'nominal':
        monkeypatch.setenv('AP_MODULE_AERO_CONTROLLER_VARIANT', variant)
    chibios_probe(tmp_path / 'rust', renode, 'aerobatics_controller', oracle,
                  {'trick72.txt': (APPLET / 'AirShow.txt').read_bytes()}, heap_limit=28672, expected_heap=65536,
                  expected_code=131072, cycles=3, timeout_ms=20000)

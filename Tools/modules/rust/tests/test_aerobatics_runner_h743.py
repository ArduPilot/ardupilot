# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Boot the real AirShow runner on H743; stationary observations are not a flight test."""

import os
import shutil
import subprocess

import pytest

from pymavlink import DFReader
from pymavlink.dialects.v20 import ardupilotmega
from test_aerobatics_path import APPLET
from test_driver_probe import ControlledPhysics
from test_runtime import ROOT
from test_threads import gen_board  # noqa: F401
from test_threads import renode  # noqa: F401
from test_threads import renode_data  # noqa: F401
from test_threads import run  # noqa: F401

import build


@pytest.mark.parametrize('mission', [False, True], ids=['rc', 'mission'])
def test_h743_runner(tmp_path, renode, mission):  # noqa: F811
    # Patch a private copy of the embedded-defaults test ELF, never the build
    # artifact used by the target-local Lua controller oracle.
    executable = tmp_path / 'arduplane'
    shutil.copyfile(os.environ.get('AP_MODULE_H743_BINARY', ROOT / 'build/MatekH743/bin/arduplane'), executable)
    defaults = tmp_path / 'runner.parm'
    defaults.write_text((ROOT / 'Tools/modules/aerobatics-oracle.parm').read_text()
                        .replace('MOD_ENABLE 0', 'MOD_ENABLE 1')
                        .replace('SCR_HEAP_SIZE 350000', 'SCR_HEAP_SIZE 65536')
                        .replace('LOG_FILE_BUFSIZE 16', 'LOG_FILE_BUFSIZE 64') + '''
MOD_HEAP_MAX 32768
MOD_CODE_MAXSIZE 131072
MOD_STACK_MAX 32768
RC7_OPTION 300
RC9_OPTION 301
RC_OVERRIDE_TIME 120
FLTMODE_CH 0
THR_FAILSAFE 0
GPS1_TYPE 2
GPS_AUTO_CONFIG 0
SERIAL3_PROTOCOL 5
SERIAL3_BAUD 230
ARSPD_BUS 1
ARSPD_SKIP_CAL 1
INS_ACCOFFS_X 0.001
INS_ACCOFFS_Y 0.001
INS_ACCOFFS_Z 0.001
INS_ACCSCAL_X 1.001
INS_ACCSCAL_Y 1.001
INS_ACCSCAL_Z 1.001
''')
    subprocess.run(['python3', str(ROOT / 'Tools/scripts/apj_tool.py'), '--set-file', str(defaults),
                    str(executable)], check=True)
    module = build.build(APPLET / 'main.rs', tmp_path / 'airshow.apmod', 'thumbv7em-none-eabihf',
                         16384, heap_size=32768)
    generated = gen_board.generate(ROOT, 'MatekH743', tmp_path / 'generated', uart_port=0, state_dir=tmp_path,
                                   num_imus=1, attachments=[
                                       {'device': 'ublox-gps', 'port': 'SERIAL3'},
                                       {'device': 'ist8310-compass', 'port': 'I2C1'},
                                       {'device': 'ms4525-airspeed', 'port': 'I2C1'},
                                   ])
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    subprocess.run(['mmd', '-i', str(sdcard), '::/APM', '::/APM/modules', '::/APM/scripts'], check=True)
    subprocess.run(['mcopy', '-i', str(sdcard), str(module), '::/APM/modules/airshow.apmod'], check=True)
    subprocess.run(['mcopy', '-i', str(sdcard), str(APPLET / 'AirShow.txt'), '::/APM/scripts/trick72.txt'], check=True)
    driver = tmp_path / 'driver.lua'
    driver_source = '''
local stage = 0
local function update()
 if stage == 0 then
  if param:get("TRIK_ENABLE") == nil then return update,100 end
  assert(param:set_and_save("TRIK_ENABLE",1))
  assert(param:set("MOD_RESTART",1))
 elseif stage == 1 then
  if param:get("TRIK1_ID") == nil then return update,100 end
  assert(param:set_and_save("TRIK1_ID",72))
  -- Stationary, disarmed runner exercise, not a flight/altitude-abort test.
  assert(param:set("AEROM_ALT_ABORT",1000))
  assert(vehicle:set_mode(7))
  assert(rc:run_aux_function(300,0))
  rc:get_channel(8):set_override(1000)
 elseif stage == 2 then
  assert(rc:run_aux_function(300,1))
 elseif stage == 3 then
  assert(rc:run_aux_function(300,2))
 elseif stage == 4 then
  assert(rc:run_aux_function(300,0))
 elseif stage == 5 then
  assert(param:set("MOD_RESTART",2))
 elseif stage == 6 then
  assert(param:get("TRIK1_ID") == 72)
  gcs:send_text(6,"H743 runner driver done")
  return
 end
 stage = stage+1
 return update,stage==4 and 20000 or 5000
end
return update,20000
'''
    if mission:
        assert driver_source.count('param:set_and_save("TRIK_ENABLE",1)') == 1
        driver_source = driver_source.replace('param:set_and_save("TRIK_ENABLE",1)',
                                              'param:set_and_save("TRIK_ENABLE",0)')
        start = driver_source.index('  if param:get("TRIK1_ID")')
        end = driver_source.index(' elseif stage == 4 then', start)
        driver_source = driver_source[:start] + '''
  assert(param:set("AEROM_ALT_ABORT",1000))
  assert(param:set("AEROM_MIS_ANGLE",90))
  assert(mission:clear())
  local m = mavlink_mission_item_int_t()
  m:command(16)
  m:x(-353632610)
  m:y(1491652300)
  m:z(40)
  m:frame(3)
  assert(mission:set_item(0,m))
  m:command(42702)
  m:param1(7)
  m:param3(10000)
  m:param4(1)
  m:x(0)
  m:y(0)
  assert(mission:set_item(1,m))
  m:command(16)
  m:x(-353632610)
  m:y(1491652300)
  assert(mission:set_item(2,m))
 elseif stage == 2 then
  assert(vehicle:set_mode(10))
 elseif stage == 3 then
  assert(mission:set_current_cmd(1))
''' + driver_source[end:]
        assert driver_source.count('assert(rc:run_aux_function(300,0))') == 1
        driver_source = driver_source.replace('assert(rc:run_aux_function(300,0))',
                                              'assert(vehicle:set_mode(7))')
        assert driver_source.count('assert(param:get("TRIK1_ID") == 72)') == 1
        driver_source = driver_source.replace('assert(param:get("TRIK1_ID") == 72)',
                                              'assert(param:get("TRIK_ENABLE") == 0)')
    driver.write_text(driver_source)
    subprocess.run(['mcopy', '-i', str(sdcard), str(driver), '::/APM/scripts/driver.lua'], check=True)
    physics = ControlledPhysics()
    physics.start()
    try:
        output = renode('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
sysbus.uart7Host CreateFileBackend @%s true
sysbus.physics Connect %u "stationary" -35.363261 149.165230 584 0 400
emulation SetGlobalAdvanceImmediately true
emulation RunFor "90"
''' % (
            executable, data['svd'], sdcard, generated['app_base'], generated['resc'],
            tmp_path / 'uart.bin', physics.port,
        ), timeout=240)
    finally:
        physics.stop()
    (tmp_path / 'renode.log').write_text(output)
    subprocess.run(['mcopy', '-s', '-i', str(sdcard), '::/APM/LOGS', str(tmp_path / 'logs')], check=True)
    parser = ardupilotmega.MAVLink(None)
    parser.robust_parsing = True
    texts = [m.text for m in parser.parse_buffer((tmp_path / 'uart.bin').read_bytes()) or []
             if m.get_type() == 'STATUSTEXT']
    stacks = []
    updates = []
    dropped = []
    for file in sorted((tmp_path / 'logs').glob('*.BIN')):
        reader = DFReader.DFReader_binary(str(file))
        while (m := reader.recv_msg()) is not None:
            if m.get_type() == 'STAK' and m.Name == 'airshow.apmod':
                stacks.append((m.Total, m.Free))
            elif m.get_type() == 'ARUN':
                updates.append((m.TimeUS, m.Us, m.Total))
            elif m.get_type() == 'DSF':
                dropped.append(m.to_dict())
    print('H743 runner messages:', texts)
    print('H743 runner stacks:', stacks, 'updates:', len(updates), updates[:3], updates[-3:])
    print('H743 logger statistics:', dropped[-3:])
    assert 'H743 runner driver done' in texts
    assert texts.count('Module: loaded 1 modules') == 3
    assert texts.count('Loaded plane_aerobatics Rust') == 3
    if mission:
        assert 'Starting Axial Roll!' in texts
        assert not any('NavScript time out' in text for text in texts)
    else:
        assert 'Trick 1 started (SuperAirShow)' in texts
        assert 'Trick aborted' in texts
    assert len(updates) > 100
    assert dropped and all(row['Dp'] == 0 for row in dropped)
    assert stacks and min(free for total, free in stacks) > 4096
    assert not any('panic' in text or 'returned -' in text or 'Aerobatics error' in text for text in texts)

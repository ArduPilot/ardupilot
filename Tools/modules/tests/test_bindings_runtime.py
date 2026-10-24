# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise generated bindings in the vehicle, optionally alongside Lua."""

import contextlib
import os
import re
import shutil
import struct

import pytest

from test_runtime import ROOT
from test_runtime import Vehicle

import build


def latest(vehicle, prefix):
    for line in reversed(vehicle.texts):
        line = line.removeprefix('Module: ')
        if line.startswith(prefix):
            return [int(number) for number in line[len(prefix):].split()]
    raise AssertionError(vehicle.diagnostic())


@pytest.mark.parametrize('lua', [False, True])
def test_native_read_bindings(tmp_path, lua):
    if lua and os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Explicit Rust-only build')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/bindings.rs', tmp_path / 'bindings.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    settings = {'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 100000} if lua else {}
    settings['SIM_ARSPD_RND'] = 0  # compare samples without injected airspeed noise
    vehicle = Vehicle(tmp_path / 'vehicle', {'bindings.apmod': image.read_bytes()}, settings)
    if lua:
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        (scripts / 'compare.lua').write_text('''local function number(v)
  if v == nil then return -9999999 end
  return math.floor(v * 1000)
end
local function update()
  gcs:send_text(6, string.format("LUA A %d %d %d", number(ahrs:get_roll_rad()),
    number(ahrs:get_pitch_rad()), number(ahrs:get_yaw_rad())))
  gcs:send_text(6, string.format("LUA E %d %d %d", number(ahrs:get_EAS2TAS()),
    number(ahrs:airspeed_estimate()), number(ahrs:get_hagl())))
  gcs:send_text(6, string.format("LUA S %d %d %d %d", vehicle:get_mode(),
    arming:is_armed() and 1 or 0, gps:num_sensors(), gps:primary_sensor()))
  gcs:send_text(6, string.format("LUA G %d %d %d %d", gps:status(0), gps:num_sats(0),
    gps:get_hdop(0), gps:have_vertical_velocity(0) and 1 or 0))
  gcs:send_text(6, string.format("LUA B %d %d %d %d", battery:num_instances(),
    battery:healthy(0) and 1 or 0, battery:pack_capacity_mah(0), number(battery:voltage(0))))
  gcs:send_text(6, string.format("LUA M %d %d %d", mission:num_commands(),
    mission:get_current_nav_index(), mission:state()))
  gcs:send_text(6, string.format("LUA R %d", rc:get_pwm(1) or 0))
  return update, 500
end
return update()
''')
    try:
        vehicle.start()
        vehicle.text('BIND R ')
        vehicle.pump(1)
        assert latest(vehicle, 'BIND S ')[1] == 0
        assert latest(vehicle, 'BIND R ')[0] == 1500
        if lua:
            vehicle.text('LUA R ')
            vehicle.pump(1)
            assert latest(vehicle, 'BIND S ') == latest(vehicle, 'LUA S ')
            assert latest(vehicle, 'BIND R ') == latest(vehicle, 'LUA R ')
            for field in ('G', 'M'):
                assert latest(vehicle, f'BIND {field} ') == latest(vehicle, f'LUA {field} ')
            native, script = latest(vehicle, 'BIND B '), latest(vehicle, 'LUA B ')
            assert native[:3] == script[:3]
            assert abs(native[3] - script[3]) <= 100  # 0.1 V, non-atomic grounded samples
            # Grounded attitude/height can vary between samples; compare at
            # 0.02 rad / 0.25 m tolerances, not as an atomic sensor snapshot.
            for a, b in zip(latest(vehicle, 'BIND A '), latest(vehicle, 'LUA A ')):
                assert abs(a - b) <= 20
            for a, b in zip(latest(vehicle, 'BIND E '), latest(vehicle, 'LUA E ')):
                assert abs(a - b) <= 250
        vehicle.link.mav.rc_channels_override_send(1, 1, 1600, 65535, 65535, 65535, 65535, 65535, 65535, 65535)
        vehicle.text('BIND R 1600', after=len(vehicle.texts))
        vehicle.arm(force=True)
        vehicle.text('BIND S 0 1 ', after=len(vehicle.texts))
        vehicle.arm(False)
        vehicle.text('BIND S 0 0 ', after=len(vehicle.texts))
        before = len(vehicle.texts)
        vehicle.parameter('MOD_RESTART', 1)
        vehicle.text('Module: loaded 1', after=before)
        vehicle.text('BIND S ', after=before)
        assert not any(re.search(r'Module: .* (panic|returned -)', line) for line in vehicle.texts)
    finally:
        vehicle.stop()


@pytest.mark.parametrize('lua', [False, True])
def test_ahrs_value_bindings(tmp_path, lua):
    if lua and os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Explicit Rust-only build')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/ahrs.rs', tmp_path / 'ahrs.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    settings = {'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 100000} if lua else {}
    vehicle = Vehicle(tmp_path / 'vehicle', {'ahrs.apmod': image.read_bytes()}, settings)
    if lua:
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        shutil.copyfile(ROOT / 'libraries/AP_Module/tests/modules/ahrs.lua', scripts / 'ahrs.lua')

    def valid_samples(prefix):
        # Wait for valid EKF position/velocity as well as attitude; a test that
        # compares only matching nil results would miss the value-return path.
        samples = {}

        def complete(message):
            if message.get_type() != 'STATUSTEXT' or not message.text.startswith(prefix):
                return False
            field, _, value = message.text[len(prefix):].partition(' ')
            if field not in ('G', 'V', 'P', 'Q', 'E'):
                return False
            if value == 'nil':
                samples.pop(field, None)
            else:
                samples[field] = [int(number) for number in value.split()]
            return len(samples) == 5

        vehicle.until(complete, timeout=20)
        return samples

    try:
        vehicle.start()
        for restart in (0, 1):
            before = len(vehicle.texts)
            if restart:
                vehicle.parameter('MOD_RESTART', restart)
                vehicle.text('Module: loaded 1', after=before)
            native = valid_samples('Module: AHRS ')
            if lua:
                script = valid_samples('LUA AHRS ')
                # Grounded samples, NOT a cross-thread atomic snapshot. Units
                # are rad/s, m/s, m, and dimensionless quaternion components.
                for field, tolerance in [('G', 20), ('V', 100), ('P', 250), ('Q', 20), ('E', 250)]:
                    assert len(native[field]) == len(script[field]) == {'Q': 4, 'E': 7}.get(field, 3)
                    assert all(abs(a - b) <= tolerance for a, b in zip(native[field], script[field])), (native, script)
        assert not any(re.search(r'Module: .* (panic|returned -)', line) for line in vehicle.texts)
    finally:
        vehicle.stop()


@pytest.mark.parametrize('lua', [False, True])
def test_enum_and_literal_bindings(tmp_path, lua):
    if lua and os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Explicit Rust-only build')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/enum_vehicle.rs', tmp_path / 'enum.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    settings = {'RC7_OPTION': 300, 'SERVO3_FUNCTION': 70, 'SIM_RC_FAIL': 0}
    if lua:
        settings.update(SCR_ENABLE=1, SCR_HEAP_SIZE=100000)
    # Two workers call pre-arm checks while restart stops and joins them.
    vehicle = Vehicle(tmp_path / 'vehicle', {'one.apmod': image.read_bytes(), 'two.apmod': image.read_bytes()}, settings)
    if lua:
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        (scripts / 'enums.lua').write_text('''local function update()
  assert(not arming:is_armed())
  local aux = rc:get_aux_cached(300)
  if aux ~= nil then
    local mode = ({[0]=0, [1]=5, [2]=4})[aux]
    if vehicle:get_mode() ~= mode then assert(vehicle:set_mode(mode)) end
    gcs:send_text(6, string.format("LUA ENUM %d %d %d %d %d %d", aux,
      SRV_Channels:find_channel(70), SRV_Channels:get_output_pwm(70), vehicle:get_mode(),
      vehicle:get_control_mode_reason(), gcs:frame_type()))
  end
  return update, 100
end
return update()
''')
    try:
        vehicle.start()
        for step, (pwm, aux, mode) in enumerate([(1000, 0, 0), (1500, 1, 5), (2000, 2, 4), (1000, 0, 0)]):
            before = len(vehicle.texts)
            vehicle.link.mav.rc_channels_override_send(1, 1, 65535, 65535, 65535, 65535, 65535, 65535, pwm, 65535)
            vehicle.text(f'ENUM {aux} 2 ', after=before)
            vehicle.until(lambda m: m.get_type() == 'HEARTBEAT' and m.custom_mode == mode)
            native = latest(vehicle, 'ENUM ')
            assert native[0] == aux and native[1] == 2 and native[3] == mode
            if step > 0:
                # ModeReason::SCRIPTING is the published firmware value 32.
                # Initial MANUAL may still have the boot reason before a change.
                assert native[4] == 32
            if lua:
                vehicle.text(f'LUA ENUM {aux} 2 ', after=before)
                native, script = latest(vehicle, 'ENUM '), latest(vehicle, 'LUA ENUM ')
                # Independent samples can straddle a throttle transition.
                assert native[:2] + native[3:] == script[:2] + script[3:]
                assert all(800 <= values[2] <= 2200 for values in (native, script))
        for _ in range(3):
            before = len(vehicle.texts)
            vehicle.parameter('MOD_RESTART', vehicle.parameter('MOD_RESTART') + 1)
            vehicle.text('Module: loaded 2 modules', after=before)
            vehicle.text('ENUM ', after=before)
        assert not any(re.search(r'Module: .* (panic|returned -|stop timeout)', line) for line in vehicle.texts)
    finally:
        vehicle.stop()


@pytest.mark.parametrize('lua', [False, True])
def test_string_bindings(tmp_path, lua):
    if lua and os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Explicit Rust-only build')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/strings.rs', tmp_path / 'strings.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    settings = {'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 100000} if lua else {}
    vehicle = Vehicle(tmp_path / 'vehicle', {'strings.apmod': image.read_bytes()}, settings)
    (vehicle.directory / 'string-test.txt').write_bytes(b'123456789')
    if lua:
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        (scripts / 'strings.lua').write_text('''local function update()
  assert(fs:crc32("string-test.txt") == uint32_t(0x2dfd2d88))
  gcs:send_text(6, "LUA STRING literal %s %n")
  gcs:send_named_float("LUA_PERR", 12.5)
  gcs:send_named_int("LUA_INT", -12345)
  gcs:send_named_string("LUA_STR", "native %s")
  return update, 200
end
return update()
''')
    try:
        vehicle.start()
        for cycle in range(3):
            before = len(vehicle.texts)
            if cycle:
                vehicle.restart(cycle)
            vehicle.text('STRING literal %s %n', after=before)
            for prefix in ['RUST'] + (['LUA'] if lua else []):
                for kind, suffix, value in [('FLOAT', 'PERR', 12.5), ('INT', 'INT', -12345),
                                            ('STRING', 'STR', 'native %s')]:
                    msg = vehicle.until(lambda m: m.get_type() == f'NAMED_VALUE_{kind}' and
                                        m.name == f'{prefix}_{suffix}')
                    assert msg.value == value
        assert not any(re.search(r'Module: .* (panic|returned -|stop timeout)', line) for line in vehicle.texts)
    finally:
        vehicle.stop()


def test_libm_lua_parity(tmp_path):
    if os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Lua comparison requires a Lua-enabled build')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/libm.rs', tmp_path / 'libm.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {'libm.apmod': image.read_bytes()},
                      {'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 100000})
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    (scripts / 'libm.lua').write_text('''local function update()
  gcs:send_named_float("LUA_SIN", math.sin(0.7))
  gcs:send_named_float("LUA_COS", math.cos(-0.7))
  gcs:send_named_float("LUA_ATAN", math.atan(0.7, -0.3))
  gcs:send_named_float("LUA_SQRT", math.sqrt(2))
  gcs:send_named_float("LUA_FMOD", math.fmod(-2.3, 0.7))
  gcs:send_named_float("LUA_FLOOR", math.floor(-1.2))
  gcs:send_named_float("LUA_CEIL", math.ceil(-1.2))
  gcs:send_named_float("LUA_TAN", math.tan(0.3))
  gcs:send_named_float("LUA_ASIN", math.asin(0.3))
  gcs:send_named_float("LUA_ACOS", math.acos(-0.3))
  gcs:send_named_float("LUA_EXP", math.exp(0.7))
  gcs:send_named_float("LUA_LOG", math.log(1.3))
  gcs:send_named_float("LUA_POW", 1.3 ^ 2.7)
  return update, 200
end
return update()
''')
    try:
        vehicle.start()
        for cycle in range(2):
            if cycle:
                vehicle.restart(cycle)
            for suffix in ('SIN', 'COS', 'ATAN', 'SQRT', 'FMOD', 'FLOOR', 'CEIL', 'TAN',
                           'ASIN', 'ACOS', 'EXP', 'LOG', 'POW'):
                values = []
                for prefix in ('LM', 'LUA'):
                    msg = vehicle.until(lambda m: m.get_type() == 'NAMED_VALUE_FLOAT' and m.name == f'{prefix}_{suffix}')
                    values.append(struct.pack('<f', msg.value))
                assert values[0] == values[1], (suffix, values)
        assert not any(re.search(r'Module: .* (panic|returned -|stop timeout)', line) for line in vehicle.texts)
    finally:
        vehicle.stop()


@pytest.mark.parametrize('lua', [False, True])
def test_rc_channel_handles(tmp_path, lua):
    if lua and os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Explicit Rust-only build')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/rc_channel.rs', tmp_path / 'rc.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    settings = {'RC7_OPTION': 300, 'RC9_OPTION': 301, 'RC9_MIN': 1100, 'RC9_MAX': 1900, 'RC9_TRIM': 1500,
                'RC9_DZ': 0, 'SIM_RC_FAIL': 0, 'RC_OVERRIDE_TIME': -1}
    if lua:
        settings.update(SCR_ENABLE=1, SCR_HEAP_SIZE=100000)
    vehicle = Vehicle(tmp_path / 'vehicle', {'rc.apmod': image.read_bytes()}, settings)
    if lua:
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        (scripts / 'rc.lua').write_text('''local cached
local function update()
  assert(not arming:is_armed())
  cached = cached or rc:find_channel_for_option(301)
  if cached then
    gcs:send_text(6, string.format("LUA RCB %d %d %d %d", cached:norm_input()*1000,
      cached:norm_input_dz()*1000, cached:norm_input_ignore_trim()*1000, cached:get_aux_switch_pos()))
  end
  return update, 100
end
return update()
''')
    try:
        vehicle.start()
        # Hold each action while comparing independent samples; clear the
        # channel explicitly below (the simulated physical RC9 input is 1000).
        for pwm, expected in ((1500, 250), (2000, 1000), (1000, -1000)):
            before = len(vehicle.texts)
            vehicle.link.mav.rc_channels_override_send(1, 1, 65535, 65535, 65535, 65535, 65535, 65535, pwm, 65535)
            vehicle.text(f'Module: RCB {expected} {expected} {expected} ', after=before)
            if lua:
                vehicle.text(f'LUA RCB {expected} {expected} {expected} ', after=before)
                assert latest(vehicle, 'RCB ') == latest(vehicle, 'LUA RCB ')
        # Both modules cache the original channel even after its option moves.
        vehicle.parameter('RC9_OPTION', 0)
        vehicle.parameter('RC10_OPTION', 301)
        before = len(vehicle.texts)
        vehicle.link.mav.rc_channels_override_send(1, 1, 65535, 65535, 65535, 65535, 65535, 65535, 1500, 65535)
        vehicle.text('Module: RCB 250 250 250 ', after=before)
        if lua:
            vehicle.text('LUA RCB 250 250 250 ', after=before)
        # Clear the override before restarting; exit does not undo shared state.
        before = len(vehicle.texts)
        vehicle.link.mav.rc_channels_override_send(1, 1, 65535, 65535, 65535, 65535, 65535, 65535, 1000, 65535)
        vehicle.text('Module: RCB -1000 -1000 -1000 ', after=before)
        vehicle.parameter('RC10_OPTION', 0)
        vehicle.parameter('RC9_OPTION', 301)
        vehicle.text('Module: RCC ')
        attempts, busy = latest(vehicle, 'RCC ')
        print(f'RC handles Lua={lua}: {busy} busy / {attempts} update attempts', flush=True)
        for cycle in range(1, 4):
            before = len(vehicle.texts)
            vehicle.restart(cycle)
            vehicle.text('Module: RCB -1000 -1000 -1000 ', after=before)
        assert not any(re.search(r'Module: .* (panic|returned -|stop timeout)', line) for line in vehicle.texts)
    finally:
        try:
            if vehicle.link is not None:
                # A failed comparison must also release the module override.
                # Teardown still terminates the isolated vehicle if it is dead.
                with contextlib.suppress(OSError, AssertionError):
                    vehicle.link.mav.rc_channels_override_send(
                        1, 1, 65535, 65535, 65535, 65535, 65535, 65535, 1000, 65535)
                    vehicle.pump(1)
        finally:
            vehicle.stop()


@pytest.mark.parametrize('lua', [False, True])
def test_location_and_mission_values(tmp_path, lua):
    if lua and os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Explicit Rust-only build')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/values_vehicle.rs', tmp_path / 'values.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {'values.apmod': image.read_bytes()},
                      {'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 100000} if lua else {})
    if lua:
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        (scripts / 'values.lua').write_text('''local function update()
  assert(not arming:is_armed())
  local item = mission:get_item(1)
  if item then
    gcs:send_text(6, string.format("LUA VM %d %d %d %d %d %d", item:command(), item:frame(),
      item:x(), item:y(), item:z(), item:param2()))
  end
  local loc = Location()
  loc:lat(-353632610); loc:lng(1491652300); loc:alt(58400)
  local other = loc:copy()
  other:offset(50, -30)
  local ne = loc:get_distance_NE(other)
  gcs:send_named_float("LUA_D", loc:get_distance(other))
  gcs:send_named_float("LUA_B", loc:get_bearing(other))
  gcs:send_named_float("LUA_N", ne:x())
  gcs:send_named_float("LUA_E", ne:y())
  return update, 200
end
return update()
''')
    try:
        vehicle.start()
        for cycle in range(3):
            start = len(vehicle.texts)
            if cycle:
                vehicle.restart(cycle)
            vehicle.text('Module: VAL M 16 3 -353632610 1491652300 42 3', after=start)
            vehicle.text('Module: VAL L ', after=start)
            vehicle.text('Module: VAL H ', after=start)
            if lua:
                vehicle.text('LUA VM 16 3 -353632610 1491652300 42 3', after=start)
                samples = {}

                def complete(message):
                    if message.get_type() == 'NAMED_VALUE_FLOAT' and message.name[:4] in ('VAL_', 'LUA_'):
                        samples[message.name] = struct.pack('<f', message.value)
                    return len(samples) == 8

                vehicle.until(complete, 10)
                for suffix in ('D', 'B', 'N', 'E'):
                    assert samples['VAL_' + suffix] == samples['LUA_' + suffix]
        assert not any(re.search(r'Module: .* (panic|returned -)', line) for line in vehicle.texts)
    finally:
        vehicle.stop()

# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Check firmware-owned Rust parameters across module reload and SITL reboot."""

import os

import pytest

from test_runtime import ROOT
from test_runtime import Vehicle

import build


class ParameterVehicle(Vehicle):
    def receive(self):
        message = super().receive()
        if message is not None and message.get_type() == 'PARAM_VALUE' and message.param_id == 'RTP_SAVED':
            self.saved_observations.append(message.param_value)
        return message


@pytest.mark.parametrize('lua', [False, True])
def test_native_parameters(tmp_path, lua):
    if lua and os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Explicit Rust-only build')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/parameters.rs', tmp_path / 'parameters.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    settings = {'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 100000} if lua else {}
    vehicle = ParameterVehicle(tmp_path / 'vehicle', {'parameters.apmod': image.read_bytes()}, settings)
    vehicle.saved_observations = []
    if lua:
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        (scripts / 'parameters.lua').write_text('''local checked = false
local function update()
  if param:get("RTP_SAVED") == nil then return update, 100 end
  if checked then
    gcs:send_text(6, "LUA PARAM coexist")
    return update, 500
  end
  assert(not param:add_table(194, "RTP_", 3))
  assert(param:add_table(196, "LUP_", 1))
  assert(param:add_param(196, 1, "VALUE", 73))
  if not checked then
    local p = Parameter()
    assert(p:init("LUP_VALUE"))
    assert(p:get() == 73)
    assert(p:set(74))
    assert(param:get("LUP_VALUE") == 74)
    assert(param:set("LUP_VALUE", 75))
    assert(p:get() == 75)
    assert(p:set(73))
    checked = true
  end
  gcs:send_text(6, "LUA PARAM coexist")
  return update, 500
end
return update()
''')
    try:
        vehicle.start()
        vehicle.text('PARAM ready')
        assert vehicle.parameter('RTP_SAVED') == 10
        assert vehicle.parameter('RTP_UNSAVED') == 20
        if lua:
            vehicle.text('LUA PARAM coexist')
            assert vehicle.parameter('LUP_VALUE') == 73
            vehicle.parameter('RTP_CMD', 3)
            vehicle.text('PARAM Lua helper')
            assert vehicle.parameter('LUP_VALUE') == 73
        vehicle.parameter('RTP_CMD', 1)
        vehicle.text('PARAM queued')
        # Only the save IO handler can send this value: no GCS read request for
        # SAVED is issued after changing it until this unsolicited report arrives.
        if 42 not in vehicle.saved_observations:
            vehicle.until(lambda msg: msg.get_type() == 'PARAM_VALUE' and
                          msg.param_id == 'RTP_SAVED' and msg.param_value == 42)
        vehicle.pump(0.2)  # allow HAL backing-file IO; reboot below verifies persistence
        assert vehicle.parameter('RTP_SAVED') == 42
        assert vehicle.parameter('RTP_UNSAVED') == 99
        vehicle.parameter('RTP_CMD', 2)
        vehicle.text('PARAM volatile')
        assert vehicle.parameter('RTP_SAVED') == 99
        assert vehicle.parameter('RTP_UNSAVED') == 21
        for generation in range(1, 11):
            mark = len(vehicle.texts)
            vehicle.restart(generation)
            vehicle.text('PARAM ready', after=mark)
            assert vehicle.parameter('RTP_SAVED') == 42
            assert vehicle.parameter('RTP_UNSAVED') == 20
        vehicle.parameter('RTP_CMD', 2)
        vehicle.text('PARAM volatile', after=mark)
        assert vehicle.parameter('RTP_SAVED') == 99
        vehicle.stop()
        mark = len(vehicle.texts)
        vehicle.start()  # same on-disk eeprom.bin, fresh process/parameter tables
        vehicle.text('PARAM ready', after=mark)
        assert vehicle.parameter('RTP_SAVED') == 42
        assert vehicle.parameter('RTP_UNSAVED') == 20
        assert not any('returned -' in text or 'panic' in text for text in vehicle.texts)
    finally:
        vehicle.stop()


def test_native_parameter_capacity(tmp_path, monkeypatch):
    monkeypatch.setenv('AP_MODULE_PARAM_MODE', 'capacity')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/parameters.rs', tmp_path / 'capacity.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {'capacity.apmod': image.read_bytes()}, {})
    try:
        vehicle.start()
        vehicle.text('PARAM capacity')
        for generation in range(1, 4):
            mark = len(vehicle.texts)
            vehicle.restart(generation)
            vehicle.text('PARAM capacity', after=mark)
        assert vehicle.parameter('R0_P01') == 1
        assert vehicle.parameter('R0_P63') == 63
        assert not any('returned -' in text or 'panic' in text for text in vehicle.texts)
    finally:
        vehicle.stop()

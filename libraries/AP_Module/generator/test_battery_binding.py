# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate the owned battery-state wire adapter and its sentinels."""

import ctypes
import os
import struct
import subprocess

import battery_binding
import bindings
import pytest


def packet():
    data = bytearray(91)
    data[0] = 2
    data[1:5] = struct.pack('<f', 24.5)
    data[5:9] = bytes([1, 2, 55, 255])
    for i in range(32):
        data[9+i*2:11+i*2] = struct.pack('<H', 3000 + i)
    data[73:75] = struct.pack('<H', 65535)
    data[75:79] = struct.pack('<f', -3.25)
    data[79:83] = struct.pack('<I', 0x7fc00000)
    data[83:87] = struct.pack('<f', 12.0)
    data[87:91] = struct.pack('<I', 0x7fc00000)
    return data


@pytest.fixture(params=[True, False])
def battery(tmp_path, request):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t) { return true; } void give() {} }; }
''',
        'AP_BattMonitor/AP_BattMonitor.h': '''#pragma once
#include <stdint.h>
#include <math.h>
struct BattMonitorScript_State {
    float voltage; bool healthy; uint8_t cell_count; uint8_t capacity_remaining_pct=255;
    uint8_t state_of_health_pct=255; uint16_t cell_voltages[32]; uint16_t cycle_count=65535;
    float current_amps=NAN, consumed_mah=NAN, consumed_wh=NAN, temperature=NAN;
};
class AP_BattMonitor {
public:
    static AP_BattMonitor *get_singleton();
    bool handle_scripting(uint8_t, const BattMonitorScript_State &);
};
''',
        'AP_BattMonitor/AP_BattMonitor_Scripting.h': '#pragma once\n',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source, library = tmp_path / 'battery.cpp', tmp_path / 'battery.so'
    source.write_text('''#include "BindingBattery.h"
#include <string.h>
static BattMonitorScript_State state;
static uint8_t index_value;
AP_BattMonitor *AP_BattMonitor::get_singleton() { static AP_BattMonitor battery; return &battery; }
bool AP_BattMonitor::handle_scripting(uint8_t index, const BattMonitorScript_State &value) {
    index_value=index; state=value; return index == 2;
}
extern "C" int call(const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    return AP_Module::binding_battery_handle_scripting(in,ni,out,no);
}
extern "C" uint8_t index_seen() { return index_value; }
extern "C" float voltage_seen() { return state.voltage; }
extern "C" uint8_t healthy_seen() { return state.healthy; }
extern "C" uint8_t cell_count_seen() { return state.cell_count; }
extern "C" uint8_t capacity_seen() { return state.capacity_remaining_pct; }
extern "C" uint8_t health_seen() { return state.state_of_health_pct; }
extern "C" uint16_t cell_seen(uint8_t i) { return state.cell_voltages[i]; }
extern "C" uint16_t cycles_seen() { return state.cycle_count; }
extern "C" uint32_t current_bits_seen() { uint32_t v; memcpy(&v, &state.current_amps, 4); return v; }
extern "C" uint32_t mah_bits_seen() { uint32_t v; memcpy(&v, &state.consumed_mah, 4); return v; }
extern "C" uint32_t wh_bits_seen() { uint32_t v; memcpy(&v, &state.consumed_wh, 4); return v; }
extern "C" uint32_t temperature_bits_seen() { uint32_t v; memcpy(&v, &state.temperature, 4); return v; }
''')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-DAP_MODULE_ENABLED=1', f'-DAP_BATTERY_SCRIPTING_ENABLED={int(request.param)}',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.voltage_seen.restype = ctypes.c_float
    lib.cell_seen.argtypes = [ctypes.c_uint8]
    lib.cell_seen.restype = ctypes.c_uint16
    lib.cycles_seen.restype = ctypes.c_uint16
    for name in ('current_bits_seen', 'mah_bits_seen', 'wh_bits_seen', 'temperature_bits_seen'):
        getattr(lib, name).restype = ctypes.c_uint32
    return lib, request.param


def test_exact_state_copy(battery):
    lib, enabled = battery
    data = packet()
    out = ctypes.create_string_buffer(1)
    assert lib.call(bytes(data), len(data), out, 1) == (0 if enabled else -3)
    assert out.raw == (b'\1' if enabled else b'\0')
    if enabled:
        assert lib.index_seen() == 2
        assert lib.voltage_seen() == 24.5
        assert lib.healthy_seen() == 1
        assert lib.cell_count_seen() == 2
        assert lib.capacity_seen() == 55
        assert lib.health_seen() == 255
        assert [lib.cell_seen(i) for i in range(32)] == list(range(3000, 3032))
        assert lib.cycles_seen() == 65535
        assert lib.current_bits_seen() == struct.unpack('<I', struct.pack('<f', -3.25))[0]
        assert lib.mah_bits_seen() == 0x7fc00000
        assert lib.wh_bits_seen() == struct.unpack('<I', struct.pack('<f', 12.0))[0]
        assert lib.temperature_bits_seen() == 0x7fc00000


def test_invalid_packet_rejected_before_backend(battery):
    lib, enabled = battery
    out = ctypes.create_string_buffer(1)
    for offset, value in [(5, 2), (6, 33)]:
        data = packet()
        data[offset] = value
        assert lib.call(bytes(data), len(data), out, 1) == (-1 if enabled else -3)
        assert out.raw == b'\0'
    data = packet()
    for ni, no in [(90, 1), (92, 1), (91, 0), (91, 2)]:
        assert lib.call(bytes(data), ni, out, no) == -1
    assert lib.call(None, 91, out, 1) == -1
    assert lib.call(bytes(data), 91, None, 1) == -1


def test_contract_and_sdk_defaults():
    spec = battery_binding.specification(10)
    assert spec['id'] == 10 and spec['name'] == 'battery.handle_scripting'
    assert 'complete-owned-copy' in spec['policy']
    rust = battery_binding.rust(spec, '')
    assert 'current_amps: None' in rust
    assert 'consumed_mah: None' in rust
    assert f'let mut input = [0u8; {battery_binding.WIRE_SIZE + 1}]' in rust

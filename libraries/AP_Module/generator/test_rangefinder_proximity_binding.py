# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate owned rangefinder state and revalidated sensor backends."""

import copy
import ctypes
import os
import struct
import subprocess

import bindings
import pytest
import rangefinder_proximity_binding as sensor_binding


@pytest.fixture(params=[True, False])
def sensors(tmp_path, request):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t) { return true; } void give() {} }; }
''',
        'AP_RangeFinder/AP_RangeFinder.h': '''#pragma once
#include <stdint.h>
class AP_RangeFinder_Backend;
class RangeFinder {
public:
    enum RotationTag { unused = 0 };
    enum class Type { NONE = 0, Lua_Scripting = 36 };
    enum class Status { NotConnected, NoData, OutOfRangeLow, OutOfRangeHigh, Good, PoweredDown };
    struct RangeFinder_State {
        float distance_m; int8_t signal_quality_pct; uint16_t voltage_mv; Status status;
        uint8_t range_valid_count; uint32_t last_reading_ms; const void *var_info;
    };
    static RangeFinder *get_singleton();
    AP_RangeFinder_Backend *get_backend(uint8_t);
};
''',
        'AP_RangeFinder/AP_RangeFinder_Backend.h': '''#pragma once
#include "AP_RangeFinder.h"
class AP_RangeFinder_Backend {
public:
    RangeFinder::Type type() const;
    RangeFinder::Status status() const;
    int32_t orientation() const;
    void get_state(RangeFinder::RangeFinder_State &);
    bool handle_script_msg(float);
    bool handle_script_msg(const RangeFinder::RangeFinder_State &);
};
''',
        'AP_Proximity/AP_Proximity.h': '''#pragma once
#include <stdint.h>
class AP_Proximity_Backend;
class AP_Proximity {
public:
    enum class Type { None = 0, Scripting = 15 };
    static AP_Proximity *get_singleton();
    AP_Proximity_Backend *get_backend(uint8_t);
};
''',
        'AP_Proximity/AP_Proximity_Backend.h': '''#pragma once
#include "AP_Proximity.h"
class AP_Proximity_Backend {
public:
    AP_Proximity::Type type() const;
    bool handle_script_distance_msg(float, float, float, bool);
    bool handle_script_3d_msg(const Vector3f &, bool);
    bool set_distance_min_max(float, float);
    bool update_virtual_boundary();
};
'''.replace('#include "AP_Proximity.h"', '#include "AP_Proximity.h"\nstruct Vector3f { float x, y, z; };'),
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)

    source = tmp_path / 'sensors.cpp'
    library = tmp_path / 'sensors.so'
    source.write_text('''#include "BindingRangeFinderProximity.h"
#include <math.h>
#include <string.h>
static RangeFinder rangefinder;
static AP_RangeFinder_Backend rangefinder_backend;
static AP_Proximity proximity;
static AP_Proximity_Backend proximity_backend;
static uint8_t present_mask = 15;
static uint8_t rangefinder_type = 36;
static uint8_t rangefinder_status = 4;
static uint8_t proximity_type = 15;
static uint32_t last_rangefinder_bits;
static uint32_t last_proximity_bits[3];
static bool last_push;
static uint32_t last_min_bits;
static uint32_t last_max_bits;
static uint32_t boundary_updates;
static uint32_t bits(float value) { uint32_t result; memcpy(&result, &value, sizeof(result)); return result; }
RangeFinder *RangeFinder::get_singleton() { return (present_mask & 1) ? &rangefinder : nullptr; }
AP_RangeFinder_Backend *RangeFinder::get_backend(uint8_t index) {
    return (present_mask & 2) && index == 0 ? &rangefinder_backend : nullptr;
}
RangeFinder::Type AP_RangeFinder_Backend::type() const {
    return static_cast<RangeFinder::Type>(rangefinder_type);
}
RangeFinder::Status AP_RangeFinder_Backend::status() const {
    return static_cast<RangeFinder::Status>(rangefinder_status);
}
int32_t AP_RangeFinder_Backend::orientation() const { return 37; }
void AP_RangeFinder_Backend::get_state(RangeFinder::RangeFinder_State &state) {
    uint32_t distance_bits = 0x7FC12345;
    memcpy(&state.distance_m, &distance_bits, sizeof(distance_bits));
    state.signal_quality_pct = -7;
    state.voltage_mv = 3300;
    state.status = static_cast<RangeFinder::Status>(rangefinder_status);
    state.range_valid_count = 9;
    state.last_reading_ms = 0x12345678;
    state.var_info = nullptr;
}
bool AP_RangeFinder_Backend::handle_script_msg(float distance) {
    last_rangefinder_bits = bits(distance);
    return true;
}
bool AP_RangeFinder_Backend::handle_script_msg(const RangeFinder::RangeFinder_State &state) {
    last_rangefinder_bits = bits(state.distance_m);
    return state.signal_quality_pct == -1 && state.voltage_mv == 3300 &&
        uint8_t(state.status) == 4 && state.range_valid_count == 9 &&
        state.last_reading_ms == 0x12345678 && state.var_info == nullptr;
}
AP_Proximity *AP_Proximity::get_singleton() { return (present_mask & 4) ? &proximity : nullptr; }
AP_Proximity_Backend *AP_Proximity::get_backend(uint8_t index) {
    return (present_mask & 8) && index == 0 ? &proximity_backend : nullptr;
}
AP_Proximity::Type AP_Proximity_Backend::type() const {
    return static_cast<AP_Proximity::Type>(proximity_type);
}
bool AP_Proximity_Backend::handle_script_distance_msg(float distance, float yaw, float pitch, bool push) {
    last_proximity_bits[0] = bits(distance);
    last_proximity_bits[1] = bits(yaw);
    last_proximity_bits[2] = bits(pitch);
    last_push = push;
    return true;
}
bool AP_Proximity_Backend::handle_script_3d_msg(const Vector3f &vector, bool push) {
    last_proximity_bits[0] = bits(vector.x);
    last_proximity_bits[1] = bits(vector.y);
    last_proximity_bits[2] = bits(vector.z);
    last_push = push;
    return true;
}
bool AP_Proximity_Backend::set_distance_min_max(float minimum, float maximum) {
    last_min_bits = bits(minimum);
    last_max_bits = bits(maximum);
    return !(minimum >= maximum);
}
bool AP_Proximity_Backend::update_virtual_boundary() { boundary_updates++; return true; }
extern "C" int rf_get(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_rangefinder_get_backend(i, n, o, z);
}
extern "C" int rf_type(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_rangefinder_type(i, n, o, z);
}
extern "C" int rf_status(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_rangefinder_status(i, n, o, z);
}
extern "C" int rf_orientation(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_rangefinder_orientation(i, n, o, z);
}
extern "C" int rf_distance(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_rangefinder_handle_script_distance(i, n, o, z);
}
extern "C" int rf_state(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_rangefinder_handle_script_state(i, n, o, z);
}
extern "C" int rf_snapshot(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_rangefinder_get_state(i, n, o, z);
}
extern "C" int prox_get(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_proximity_get_backend(i, n, o, z);
}
extern "C" int prox_type(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_proximity_type(i, n, o, z);
}
extern "C" int prox_distance(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_proximity_handle_script_distance(i, n, o, z);
}
extern "C" int prox_3d(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_proximity_handle_script_3d(i, n, o, z);
}
extern "C" int prox_min_max(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_proximity_set_distance_min_max(i, n, o, z);
}
extern "C" int prox_boundary(const uint8_t *i, uint32_t n, uint8_t *o, uint32_t z) {
    return AP_Module::binding_proximity_update_virtual_boundary(i, n, o, z);
}
extern "C" void configure(uint8_t mask, uint8_t rf_type_value, uint8_t rf_status_value,
                           uint8_t prox_type_value) {
    present_mask = mask; rangefinder_type = rf_type_value;
    rangefinder_status = rf_status_value; proximity_type = prox_type_value;
}
extern "C" uint32_t rf_bits() { return last_rangefinder_bits; }
extern "C" uint32_t prox_bits(uint8_t index) { return last_proximity_bits[index]; }
extern "C" bool prox_push() { return last_push; }
extern "C" uint32_t min_bits() { return last_min_bits; }
extern "C" uint32_t max_bits() { return last_max_bits; }
extern "C" uint32_t boundary_count() { return boundary_updates; }
''')
    enabled = int(request.param)
    subprocess.run([
        os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
        '-DAP_MODULE_ENABLED=1', '-DAP_SCRIPTING_ENABLED=1',
        f'-DAP_RANGEFINDER_ENABLED={enabled}', f'-DHAL_PROXIMITY_ENABLED={enabled}',
        '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source), '-o', str(library),
    ], check=True)
    lib = ctypes.CDLL(str(library))
    for name in ('rf_get', 'rf_type', 'rf_status', 'rf_orientation', 'rf_distance', 'rf_state', 'rf_snapshot',
                 'prox_get', 'prox_type',
                 'prox_distance', 'prox_3d', 'prox_min_max', 'prox_boundary'):
        getattr(lib, name).argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.configure.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8]
    for name in ('rf_bits', 'min_bits', 'max_bits', 'boundary_count'):
        getattr(lib, name).restype = ctypes.c_uint32
    lib.prox_bits.argtypes = [ctypes.c_uint8]
    lib.prox_bits.restype = ctypes.c_uint32
    lib.prox_push.restype = ctypes.c_bool
    return lib, request.param


def call(lib, name, packet):
    output = ctypes.create_string_buffer(1)
    status = getattr(lib, name)(packet, len(packet), output, 1)
    return status, output.raw[0]


def test_selectors_and_revalidation(sensors):
    lib, enabled = sensors
    lib.configure(15, 36, 4, 15)
    expected = 0 if enabled else -3
    assert call(lib, 'rf_get', b'\0') == (expected, 1 if enabled else 0)
    assert call(lib, 'prox_get', b'\0') == (expected, 1 if enabled else 0)
    assert call(lib, 'rf_get', b'\1') == (expected, 0)
    assert call(lib, 'prox_get', b'\1') == (expected, 0)
    assert call(lib, 'rf_type', b'\0') == (expected, 36 if enabled else 0)
    assert call(lib, 'rf_status', b'\0') == (expected, 4 if enabled else 0)
    assert call(lib, 'prox_type', b'\0') == (expected, 15 if enabled else 0)

    lib.configure(5, 36, 4, 15)
    assert call(lib, 'rf_type', b'\0') == (-3, 0)
    assert call(lib, 'prox_type', b'\0') == (-3, 0)


def test_rangefinder_distance_and_owned_state(sensors):
    lib, enabled = sensors
    lib.configure(15, 36, 4, 15)
    nan_bits = 0x7FC12345
    status, value = call(lib, 'rf_distance', b'\0' + struct.pack('<I', nan_bits))
    assert (status, value) == ((0, 1) if enabled else (-3, 0))
    if enabled:
        assert lib.rf_bits() == nan_bits

    packet = bytearray(1 + sensor_binding.STATE_WIRE_SIZE)
    packet[0] = 0
    packet[1:5] = struct.pack('<f', 2.75)
    packet[5] = 0xFF
    packet[6:8] = struct.pack('<H', 3300)
    packet[8] = 4
    packet[9] = 9
    packet[10:14] = struct.pack('<I', 0x12345678)
    assert call(lib, 'rf_state', bytes(packet)) == ((0, 1) if enabled else (-3, 0))
    packet[8] = 6
    assert call(lib, 'rf_state', bytes(packet)) == ((-1, 0) if enabled else (-3, 0))


def test_rangefinder_snapshot_is_owned_and_revalidated(sensors):
    lib, enabled = sensors
    lib.configure(15, 36, 4, 15)
    output = ctypes.create_string_buffer(sensor_binding.STATE_WIRE_SIZE)
    status = lib.rf_snapshot(b'\0', 1, output, sensor_binding.STATE_WIRE_SIZE)
    expected = (struct.pack('<I', 0x7FC12345) + bytes([0xF9]) + struct.pack('<H', 3300) +
                bytes([4, 9]) + struct.pack('<I', 0x12345678))
    assert status == (0 if enabled else -3)
    assert output.raw == (expected if enabled else bytes(sensor_binding.STATE_WIRE_SIZE))

    lib.configure(13, 36, 4, 15)
    output.raw = b'\xA5' * sensor_binding.STATE_WIRE_SIZE
    assert lib.rf_snapshot(b'\0', 1, output, sensor_binding.STATE_WIRE_SIZE) == -3
    assert output.raw == bytes(sensor_binding.STATE_WIRE_SIZE)

    lib.configure(15, 36, 6, 15)
    output.raw = b'\xA5' * sensor_binding.STATE_WIRE_SIZE
    assert lib.rf_snapshot(b'\0', 1, output, sensor_binding.STATE_WIRE_SIZE) == (-4 if enabled else -3)
    assert output.raw == bytes(sensor_binding.STATE_WIRE_SIZE)


def test_proximity_forwarding_and_native_returns(sensors):
    lib, enabled = sensors
    lib.configure(15, 36, 4, 15)
    fields = [0x7FC01234, struct.unpack('<I', struct.pack('<f', -22.5))[0],
              struct.unpack('<I', struct.pack('<f', 11.25))[0]]
    packet = b'\0' + b''.join(struct.pack('<I', value) for value in fields) + b'\1'
    assert call(lib, 'prox_distance', packet) == ((0, 1) if enabled else (-3, 0))
    if enabled:
        assert [lib.prox_bits(index) for index in range(3)] == fields
        assert lib.prox_push()

    assert call(lib, 'prox_3d', packet) == ((0, 1) if enabled else (-3, 0))

    min_max = b'\0' + struct.pack('<ff', 5.0, 4.0)
    assert call(lib, 'prox_min_max', min_max) == ((0, 0) if enabled else (-3, 0))
    nan_min_max = b'\0' + struct.pack('<If', 0x7FC05678, 4.0)
    assert call(lib, 'prox_min_max', nan_min_max) == ((0, 1) if enabled else (-3, 0))
    assert call(lib, 'prox_boundary', b'\0') == ((0, 1) if enabled else (-3, 0))
    if enabled:
        assert lib.boundary_count() == 1


def test_invalid_wire_is_rejected(sensors):
    lib, enabled = sensors
    lib.configure(15, 36, 6, 15)
    assert call(lib, 'rf_status', b'\0') == ((-4, 0) if enabled else (-3, 0))
    invalid_bool = b'\0' + struct.pack('<fff', 1.0, 2.0, 3.0) + b'\2'
    assert call(lib, 'prox_distance', invalid_bool) == ((-1, 0) if enabled else (-3, 0))
    output = ctypes.create_string_buffer(1)
    assert lib.rf_distance(b'\0\0\0\0', 4, output, 1) == -1
    assert output.raw == b'\0'


def test_contract_and_sdk_are_owned(tmp_path):
    spec = sensor_binding.select(bindings.read_ir(), 40)
    assert [method['id'] for method in spec['methods']] == list(range(40, 53))
    assert all('revalidate-every-call' in method['policy'] for method in spec['methods'])
    rust = sensor_binding.rust(spec, '')
    assert '*mut' not in rust and '*const' not in rust
    assert 'pub struct RangeFinderBackend { index: u8 }' in rust
    assert 'pub struct ProximityBackend { index: u8 }' in rust
    assert "impl crate::bindings::Rangefinder<'_, '_>" in rust
    assert "impl crate::bindings::Proximity<'_, '_>" in rust
    assert 'pub fn rangefinder_get_backend' not in rust
    assert 'pub fn proximity_get_backend' not in rust
    assert 'pub fn get_state(&self' in rust
    assert 'pub fn distance(&self' in rust
    assert 'pub fn signal_quality(&self' in rust
    assert f'let mut input = [0u8; {sensor_binding.STATE_WIRE_SIZE + 1}]' in rust
    assert f'let mut output = [0u8; {sensor_binding.STATE_WIRE_SIZE}]' in rust
    source = tmp_path / 'sensors.rs'
    imports = '\n'.join(
        f'    pub static BINDING_{binding_id}: Import = Import;' for binding_id in range(40, 53))
    source.write_text('''#![no_std]
pub enum Error { Host }
pub type Result<T=()> = core::result::Result<T, Error>;
pub fn result(value: i32) -> Result { if value == 0 { Ok(()) } else { Err(Error::Host) } }
pub struct Api { pub binding_call: Option<unsafe extern "C" fn(
    *mut core::ffi::c_void, u32, *const u8, u32, *mut u8, u32) -> i32> }
pub struct Context<'a> { raw: *mut core::ffi::c_void, api: &'a Api }
pub mod bindings {
    use super::Context;
    pub struct Rangefinder<'context, 'api> { pub(crate) context: &'context Context<'api> }
    pub struct Proximity<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
pub struct Vector3f { pub x: f32, pub y: f32, pub z: f32 }
impl Vector3f { pub(crate) fn encode(&self) -> [u8; 12] {
    let mut value = [0; 12]; value[0..4].copy_from_slice(&self.x.to_le_bytes());
    value[4..8].copy_from_slice(&self.y.to_le_bytes());
    value[8..12].copy_from_slice(&self.z.to_le_bytes()); value
} }
pub mod enums { pub struct Rotation(pub i32); }
pub mod imports {
    pub struct Import;
    impl Import { pub const fn slot(&self) -> u32 { 0 } }
''' + imports + '''
}
mod generated {
''' + rust + '\n}\n')
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--crate-type', 'lib', '--edition', '2021',
                    str(source), '-o', str(tmp_path / 'sensors.rlib')], check=True)


def test_ir_selection_fails_closed():
    ir = bindings.read_ir()
    assert sensor_binding.select(ir, 40)['methods'][0]['name'] == 'RangeFinder.get_backend'

    mutations = []
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['singletons'] if item['name'] == sensor_binding.RANGEFINDER_OWNER)
    next(method for method in owner['methods'] if method['name'] == 'get_backend')['arguments'][0]['skip_check'] = False
    mutations.append((changed, 'RangeFinder get_backend'))
    changed = copy.deepcopy(ir)
    backend = next(item for item in changed['ap_objects']
                   if item['name'] == sensor_binding.RANGEFINDER_BACKEND)
    backend['aliases'][0]['manual'] = False
    mutations.append((changed, 'RangeFinder handle_script_msg'))
    changed = copy.deepcopy(ir)
    backend = next(item for item in changed['ap_objects']
                   if item['name'] == sensor_binding.RANGEFINDER_BACKEND)
    next(method for method in backend['methods'] if method['name'] == 'get_state')['arguments'][0][
        'reference'] = False
    mutations.append((changed, 'RangeFinder backend methods'))
    changed = copy.deepcopy(ir)
    backend = next(item for item in changed['ap_objects']
                   if item['name'] == sensor_binding.RANGEFINDER_BACKEND)
    next(method for method in backend['methods'] if method['name'] == 'distance')['return']['kind'] = 'double'
    mutations.append((changed, 'RangeFinder backend methods'))
    changed = copy.deepcopy(ir)
    backend = next(item for item in changed['ap_objects']
                   if item['name'] == sensor_binding.RANGEFINDER_BACKEND)
    next(method for method in backend['methods'] if method['name'] == 'signal_quality_pct')['rename'] = None
    mutations.append((changed, 'RangeFinder backend methods'))
    changed = copy.deepcopy(ir)
    state = next(item for item in changed['userdata'] if item['name'] == sensor_binding.RANGEFINDER_STATE)
    next(field for field in state['fields'] if field['name'] == 'status')['write'] = False
    mutations.append((changed, 'RangeFinder state'))
    changed = copy.deepcopy(ir)
    backend = next(item for item in changed['ap_objects'] if item['name'] == sensor_binding.PROXIMITY_BACKEND)
    next(method for method in backend['methods']
         if method['name'] == 'handle_script_distance_msg')['arguments'][3]['kind'] = 'uint8_t'
    mutations.append((changed, 'Proximity backend methods'))

    for changed, error in mutations:
        with pytest.raises(ValueError, match=error):
            sensor_binding.select(changed, 40)

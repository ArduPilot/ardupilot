# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise owned ESC updates and validity-aware servo telemetry copies."""

import copy
import ctypes
import os
import struct
import subprocess

import bindings
import pytest
import telemetry_binding


@pytest.fixture(params=[True, False])
def telemetry_host(tmp_path, request):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t) { return true; } void give() {} }; }
''',
        'AP_ESC_Telem/AP_ESC_Telem.h': '''#pragma once
#include <stdint.h>
#if HAL_WITH_ESC_TELEM
#define ESC_TELEM_MAX_ESCS 4
class AP_ESC_Telem_Backend { public:
    struct TelemetryData { int16_t temperature_cdeg; float voltage; float current; };
    enum TelemetryType { TEMPERATURE=1U<<0, VOLTAGE=1U<<2, CURRENT=1U<<3 };
};
class AP_ESC_Telem { public:
    static AP_ESC_Telem *get_singleton();
    void update_telem_data(uint8_t, const AP_ESC_Telem_Backend::TelemetryData &, uint16_t);
};
#endif
''',
        'AP_Servo_Telem/AP_Servo_Telem.h': '''#pragma once
#include <stdint.h>
#if AP_SERVO_TELEM_ENABLED
class AP_Servo_Telem { public:
    struct TelemetryData {
        float command_position, measured_position, force, speed, voltage, current;
        uint8_t duty_cycle; int16_t motor_temperature_cdeg, pcb_temperature_cdeg; uint8_t status_flags;
        enum Types { COMMANDED_POSITION=1U<<0, MEASURED_POSITION=1U<<1, FORCE=1U<<2,
            SPEED=1U<<3, VOLTAGE=1U<<4, CURRENT=1U<<5, DUTY_CYCLE=1U<<6,
            MOTOR_TEMP=1U<<7, PCB_TEMP=1U<<8, STATUS=1U<<9 };
        uint16_t present_types;
        bool present(uint16_t mask) const { return (present_types & mask) == mask; }
    };
    static AP_Servo_Telem *get_singleton();
    bool get_telem(uint8_t, TelemetryData &);
    void update_telem_data(uint8_t, const TelemetryData &);
};
#endif
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source, library = tmp_path / 'telemetry.cpp', tmp_path / 'telemetry.so'
    source.write_text('''#include "BindingTelemetry.h"
#include <string.h>
static bool esc_present = true;
static bool servo_present = true;
static bool servo_active = true;
static bool servo_measured_present = true;
static uint32_t servo_value_bits;
#if AP_SERVO_TELEM_ENABLED
static uint8_t servo_seen_index;
static AP_Servo_Telem::TelemetryData servo_seen_data;
#endif
#if HAL_WITH_ESC_TELEM
static AP_ESC_Telem esc;
static uint8_t seen_index;
static AP_ESC_Telem_Backend::TelemetryData seen_data;
static uint16_t seen_mask;
AP_ESC_Telem *AP_ESC_Telem::get_singleton() { return esc_present ? &esc : nullptr; }
void AP_ESC_Telem::update_telem_data(uint8_t index, const AP_ESC_Telem_Backend::TelemetryData &data,
                                    uint16_t mask) {
    seen_index=index; seen_data=data; seen_mask=mask;
}
#endif
#if AP_SERVO_TELEM_ENABLED
static AP_Servo_Telem servo;
AP_Servo_Telem *AP_Servo_Telem::get_singleton() { return servo_present ? &servo : nullptr; }
bool AP_Servo_Telem::get_telem(uint8_t index, TelemetryData &data) {
    if (!servo_active || index != 2) { return false; }
    memcpy(&data.measured_position, &servo_value_bits, sizeof(data.measured_position));
    data.present_types = servo_measured_present ? TelemetryData::MEASURED_POSITION : 0;
    return true;
}
void AP_Servo_Telem::update_telem_data(uint8_t index, const TelemetryData &data) {
    servo_seen_index=index; servo_seen_data=data;
}
#endif
extern "C" int esc_update(const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    return AP_Module::binding_esc_telem_update_telem_data(in,ni,out,no);
}
extern "C" int servo_get(const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    return AP_Module::binding_servo_telem_get_telem(in,ni,out,no);
}
extern "C" int servo_update(const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    return AP_Module::binding_servo_telem_update_telem_data(in,ni,out,no);
}
extern "C" void configure(bool ep, bool sp, bool active, bool measured, uint32_t bits) {
    esc_present=ep; servo_present=sp; servo_active=active; servo_measured_present=measured;
    servo_value_bits=bits;
}
extern "C" uint32_t esc_seen(unsigned field) {
#if HAL_WITH_ESC_TELEM
    if (field == 0) { return seen_index; }
    if (field == 1) { return seen_mask; }
    if (field == 2) { return uint16_t(seen_data.temperature_cdeg); }
    uint32_t bits; memcpy(&bits, field == 3 ? &seen_data.voltage : &seen_data.current, sizeof(bits));
    return bits;
#else
    (void)field; return 0;
#endif
}
extern "C" uint32_t servo_seen(unsigned field) {
#if AP_SERVO_TELEM_ENABLED
    if (field == 0) { return servo_seen_index; }
    if (field == 1) { return servo_seen_data.present_types; }
    if (field == 8) { return servo_seen_data.duty_cycle; }
    if (field == 9) { return uint16_t(servo_seen_data.motor_temperature_cdeg); }
    if (field == 10) { return uint16_t(servo_seen_data.pcb_temperature_cdeg); }
    if (field == 11) { return servo_seen_data.status_flags; }
    const float *values=&servo_seen_data.command_position; uint32_t bits; memcpy(&bits,&values[field-2],4); return bits;
#else
    (void)field; return 0;
#endif
}
''')
    enabled = int(request.param)
    subprocess.run([
        os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
        '-DAP_MODULE_ENABLED=1', f'-DHAL_WITH_ESC_TELEM={enabled}',
        f'-DAP_SERVO_TELEM_ENABLED={enabled}', '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
        str(source), '-o', str(library),
    ], check=True)
    lib = ctypes.CDLL(str(library))
    for name in ('esc_update', 'servo_get', 'servo_update'):
        getattr(lib, name).argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
        getattr(lib, name).restype = ctypes.c_int
    lib.configure.argtypes = [ctypes.c_bool, ctypes.c_bool, ctypes.c_bool, ctypes.c_bool, ctypes.c_uint32]
    lib.esc_seen.argtypes = [ctypes.c_uint]
    lib.esc_seen.restype = ctypes.c_uint32
    lib.servo_seen.argtypes = [ctypes.c_uint]
    lib.servo_seen.restype = ctypes.c_uint32
    return lib, request.param


def esc_packet(index=1, present=7, temperature=-1234, voltage_bits=0x7FC12345,
               current_bits=0x7F800000):
    return bytes([index, present]) + struct.pack('<hII', temperature, voltage_bits, current_bits)


def test_esc_owned_values_and_native_presence(telemetry_host):
    lib, enabled = telemetry_host
    packet = esc_packet()
    assert lib.esc_update(packet, len(packet), None, 0) == (0 if enabled else -3)
    if not enabled:
        return
    assert [lib.esc_seen(index) for index in range(5)] == [1, 0x0D, 0xFB2E, 0x7FC12345, 0x7F800000]

    for compact, native in ((1, 1), (2, 4), (4, 8)):
        packet = esc_packet(present=compact)
        assert lib.esc_update(packet, len(packet), None, 0) == 0
        assert lib.esc_seen(1) == native


def test_esc_bounds_and_invalid_wire(telemetry_host):
    lib, enabled = telemetry_host
    for packet in (esc_packet(present=0), esc_packet(present=8)):
        assert lib.esc_update(packet, len(packet), None, 0) == -1
    packet = esc_packet()
    assert lib.esc_update(packet, len(packet) - 1, None, 0) == -1
    assert lib.esc_update(packet, len(packet), ctypes.create_string_buffer(1), 0) == -1
    if enabled:
        assert lib.esc_update(esc_packet(index=4), telemetry_binding.ESC_WIRE_SIZE, None, 0) == -1
        lib.configure(False, True, True, True, 0)
        assert lib.esc_update(packet, len(packet), None, 0) == -3


def test_servo_presence_and_measured_validity(telemetry_host):
    lib, enabled = telemetry_host
    bits = 0xFFC12345
    output = ctypes.create_string_buffer(telemetry_binding.SERVO_WIRE_SIZE)
    lib.configure(True, True, True, True, bits)
    assert lib.servo_get(b'\2', 1, output, len(output)) == (0 if enabled else -3)
    assert output.raw == ((b'\1\1' + struct.pack('<I', bits)) if enabled else bytes(6))
    if not enabled:
        return

    lib.configure(True, True, True, False, bits)
    assert lib.servo_get(b'\2', 1, output, len(output)) == 0
    assert output.raw == b'\1' + bytes(5)
    lib.configure(True, True, False, True, bits)
    assert lib.servo_get(b'\2', 1, output, len(output)) == 0
    assert output.raw == bytes(6)
    lib.configure(True, False, True, True, bits)
    assert lib.servo_get(b'\2', 1, output, len(output)) == -3
    assert output.raw == bytes(6)


def test_servo_wire_validation(telemetry_host):
    lib, _ = telemetry_host
    output = ctypes.create_string_buffer(telemetry_binding.SERVO_WIRE_SIZE)
    assert lib.servo_get(None, 1, output, len(output)) == -1
    assert lib.servo_get(b'\0', 0, output, len(output)) == -1
    assert lib.servo_get(b'\0', 1, output, len(output) - 1) == -1


def test_servo_update_owned_value(telemetry_host):
    lib, enabled = telemetry_host
    values = [0x3F800000, 0x40000000, 0x40400000, 0x40800000, 0x40A00000, 0x40C00000]
    packet = bytes([3]) + struct.pack('<H', 0x3FF) + b''.join(struct.pack('<I', v) for v in values)
    packet += bytes([72]) + struct.pack('<hhB', -1200, 3400, 0xA5)
    assert len(packet) == telemetry_binding.SERVO_UPDATE_WIRE_SIZE
    assert lib.servo_update(packet, len(packet), None, 0) == (0 if enabled else -3)
    if enabled:
        assert [lib.servo_seen(i) for i in range(12)] == [3, 0x3FF] + values + [72, 0xFB50, 3400, 0xA5]
    empty = bytearray(packet)
    empty[1:3] = b'\0\0'
    assert lib.servo_update(bytes(empty), len(empty), None, 0) == -1


def test_ir_selection_fails_closed():
    ir = bindings.read_ir()
    spec = telemetry_binding.select(ir, 90)
    assert [method['name'] for method in spec['methods']] == [
        'esc_telem.update_telem_data', 'servo_telem.get_telem', 'servo_telem.update_telem_data']

    mutations = []
    changed = copy.deepcopy(ir)
    next(item for item in changed['singletons'] if item['name'] == telemetry_binding.ESC_OWNER)['depends'] = None
    mutations.append((changed, 'AP_ESC_Telem metadata'))
    changed = copy.deepcopy(ir)
    esc = next(item for item in changed['singletons'] if item['name'] == telemetry_binding.ESC_OWNER)
    next(method for method in esc['methods'] if method['name'] == 'update_telem_data')['arguments'][2]['skip_check'] = False
    mutations.append((changed, 'AP_ESC_Telem.update_telem_data method'))
    changed = copy.deepcopy(ir)
    value = next(item for item in changed['userdata'] if item['name'] == telemetry_binding.ESC_VALUE)
    next(field for field in value['fields'] if field['name'] == 'current')['write'] = False
    mutations.append((changed, 'AP_ESC_Telem_Backend::TelemetryData.current field'))
    changed = copy.deepcopy(ir)
    servo = next(item for item in changed['singletons'] if item['name'] == telemetry_binding.SERVO_OWNER)
    next(method for method in servo['methods'] if method['name'] == 'get_telem')['return']['kind'] = 'uint8_t'
    mutations.append((changed, 'AP_Servo_Telem.get_telem method'))
    changed = copy.deepcopy(ir)
    value = next(item for item in changed['userdata'] if item['name'] == telemetry_binding.SERVO_VALUE)
    field = next(field for field in value['fields'] if field['name'] == 'measured_position')
    field['valid_mask_value'] = 'wrong'
    mutations.append((changed, 'AP_Servo_Telem::TelemetryData.measured_position field'))

    for changed, error in mutations:
        with pytest.raises(ValueError, match=error):
            telemetry_binding.select(changed, 90)


def test_generated_rust_compiles_and_uses_options(tmp_path):
    rust = telemetry_binding.rust(telemetry_binding.specification(90), '')
    assert '*mut' not in rust and '*const' not in rust
    assert "impl crate::bindings::EscTelem<'_, '_>" in rust
    assert "impl crate::bindings::ServoTelem<'_, '_>" in rust
    assert 'pub fn update_telem_data(&self, esc_index: u8' in rust
    assert 'pub fn get_telem(&self, servo_index: u8)' in rust
    assert 'temperature_cdeg: Option<i16>' in rust
    assert 'measured_position: Option<f32>' in rust
    assert 'if input[1] == 0 { return Err(Error::Invalid); }' in rust
    source = tmp_path / 'telemetry.rs'
    source.write_text('''#![no_std]
#[derive(Clone, Copy)] pub enum Error { Invalid, Host }
pub type Result<T=()> = core::result::Result<T, Error>;
pub fn result(value: i32) -> Result { if value == 0 { Ok(()) } else { Err(Error::Host) } }
pub struct Api { pub binding_call: Option<unsafe extern "C" fn(
    *mut core::ffi::c_void, u32, *const u8, u32, *mut u8, u32) -> i32> }
pub struct Context<'a> { raw: *mut core::ffi::c_void, api: &'a Api }
pub mod bindings {
    use super::Context;
    pub struct EscTelem<'context, 'api> { pub(crate) context: &'context Context<'api> }
    pub struct ServoTelem<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
pub mod imports { pub struct Import; impl Import { pub const fn slot(&self)->u32 { 0 } }
pub static BINDING_90: Import = Import;
pub static BINDING_91: Import = Import;
pub static BINDING_92: Import = Import;
}
mod generated {
''' + rust + '\n}\n')
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--crate-type', 'lib', '--edition', '2021',
                    str(source), '-o', str(tmp_path / 'telemetry.rlib')], check=True)

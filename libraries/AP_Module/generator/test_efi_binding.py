# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate the owned EFI wire state and index-revalidated backend."""

import copy
import ctypes
import os
import struct
import subprocess

import bindings
import efi_binding
import pytest


def full_packet():
    data = bytearray(efi_binding.STATE_WIRE_SIZE + 1)
    data[0] = 0
    data[1:5] = struct.pack('<I', 0x12345678)
    data[5:15] = bytes([3, 1, 2, 5, 3, 3, 2, 2, 2, 77])
    data[15:19] = struct.pack('<I', 9876)
    for index, offset in enumerate(range(19, 59, 4), 1):
        data[offset:offset+4] = struct.pack('<f', index + 0.25)
    data[59:62] = bytes([66, 4, 3])
    for index, offset in enumerate(range(62, 90, 4), 11):
        data[offset:offset+4] = struct.pack('<f', index + 0.25)
    for index, offset in enumerate(range(90, 102, 4), 18):
        data[offset:offset+4] = struct.pack('<f', index + 0.25)
    return data


def default_packet():
    data = bytearray(efi_binding.STATE_WIRE_SIZE + 1)
    nan = struct.pack('<I', 0x7fc00000)
    for offset in list(range(19, 59, 4)) + list(range(62, 90, 4)) + [94, 98]:
        data[offset:offset+4] = nan
    data[90:94] = struct.pack('<f', -1.0)
    return data


@pytest.fixture(params=[True, False])
def efi(tmp_path, request):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t) { return true; } void give() {} }; }
''',
        'AP_EFI/AP_EFI.h': '''#pragma once
#include <stdint.h>
enum class Engine_State : uint8_t { STOPPED, STARTING, RUNNING, FAULT };
enum class Crankshaft_Sensor_Status : uint8_t { NOT_SUPPORTED, OK, ERROR };
enum class Temperature_Status : uint8_t { NOT_SUPPORTED, OK, BELOW_NOMINAL, ABOVE_NOMINAL, OVERHEATING,
                                          EGT_ABOVE_NOMINAL };
enum class Fuel_Pressure_Status : uint8_t { NOT_SUPPORTED, OK, BELOW_NOMINAL, ABOVE_NOMINAL };
enum class Oil_Pressure_Status : uint8_t { NOT_SUPPORTED, OK, BELOW_NOMINAL, ABOVE_NOMINAL };
enum class Detonation_Status : uint8_t { NOT_SUPPORTED, NOT_OBSERVED, OBSERVED };
enum class Misfire_Status : uint8_t { NOT_SUPPORTED, NOT_OBSERVED, OBSERVED };
enum class Debris_Status : uint8_t { NOT_SUPPORTED, NOT_DETECTED, DETECTED };
enum class Spark_Plug_Usage : uint8_t { SINGLE, FIRST_ACTIVE, SECOND_ACTIVE, BOTH_ACTIVE };
struct Cylinder_Status { float ignition_timing_deg, injection_time_ms, cylinder_head_temperature;
    float cylinder_head_temperature2, exhaust_gas_temperature, exhaust_gas_temperature2, lambda_coefficient; };
struct EFI_State {
    uint32_t last_updated_ms; Engine_State engine_state; bool general_error;
    Crankshaft_Sensor_Status crankshaft_sensor_status; Temperature_Status temperature_status;
    Fuel_Pressure_Status fuel_pressure_status; Oil_Pressure_Status oil_pressure_status;
    Detonation_Status detonation_status; Misfire_Status misfire_status; Debris_Status debris_status;
    uint8_t engine_load_percent; uint32_t engine_speed_rpm;
    float spark_dwell_time_ms, atmospheric_pressure_kpa, intake_manifold_pressure_kpa;
    float intake_manifold_temperature, coolant_temperature, oil_pressure, oil_temperature, fuel_pressure;
    float fuel_consumption_rate_cm3pm, estimated_consumed_fuel_volume_cm3;
    uint8_t throttle_position_percent, ecu_index; Spark_Plug_Usage spark_plug_usage;
    Cylinder_Status cylinder_status; float ignition_voltage=-1, throttle_out, pt_compensation;
};
class AP_EFI_Backend { public: bool handle_scripting(const EFI_State &); };
class AP_EFI { public: static AP_EFI *get_singleton(); AP_EFI_Backend *get_backend(uint8_t);
    void get_state(EFI_State &); };
''',
        'AP_EFI/AP_EFI_Scripting.h': '''#pragma once
#if !AP_EFI_SCRIPTING_ENABLED
#error "Disabled EFI backend must not be included"
#endif
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source, library = tmp_path / 'efi.cpp', tmp_path / 'efi.so'
    source.write_text('''#include "BindingEFI.h"
#include <math.h>
static AP_EFI frontend;
static AP_EFI_Backend backend;
static bool frontend_present = true;
static unsigned expected_mode;
static EFI_State last_state;
AP_EFI *AP_EFI::get_singleton() { return frontend_present ? &frontend : nullptr; }
AP_EFI_Backend *AP_EFI::get_backend(uint8_t index) { return index == 0 ? &backend : nullptr; }
void AP_EFI::get_state(EFI_State &state) { state = last_state; }
static bool equal(float value, float expected) { return value == expected; }
bool AP_EFI_Backend::handle_scripting(const EFI_State &s) {
    last_state = s;
    if (expected_mode == 0) {
        return s.last_updated_ms == 0x12345678 && uint8_t(s.engine_state) == 3 && s.general_error &&
            uint8_t(s.crankshaft_sensor_status) == 2 && uint8_t(s.temperature_status) == 5 &&
            uint8_t(s.fuel_pressure_status) == 3 && uint8_t(s.oil_pressure_status) == 3 &&
            uint8_t(s.detonation_status) == 2 && uint8_t(s.misfire_status) == 2 &&
            uint8_t(s.debris_status) == 2 && s.engine_load_percent == 77 && s.engine_speed_rpm == 9876 &&
            equal(s.spark_dwell_time_ms, 1.25f) && equal(s.atmospheric_pressure_kpa, 2.25f) &&
            equal(s.intake_manifold_pressure_kpa, 3.25f) && equal(s.intake_manifold_temperature, 4.25f) &&
            equal(s.coolant_temperature, 5.25f) && equal(s.oil_pressure, 6.25f) &&
            equal(s.oil_temperature, 7.25f) && equal(s.fuel_pressure, 8.25f) &&
            equal(s.fuel_consumption_rate_cm3pm, 9.25f) &&
            equal(s.estimated_consumed_fuel_volume_cm3, 10.25f) && s.throttle_position_percent == 66 &&
            s.ecu_index == 4 && uint8_t(s.spark_plug_usage) == 3 &&
            equal(s.cylinder_status.ignition_timing_deg, 11.25f) &&
            equal(s.cylinder_status.injection_time_ms, 12.25f) &&
            equal(s.cylinder_status.cylinder_head_temperature, 13.25f) &&
            equal(s.cylinder_status.cylinder_head_temperature2, 14.25f) &&
            equal(s.cylinder_status.exhaust_gas_temperature, 15.25f) &&
            equal(s.cylinder_status.exhaust_gas_temperature2, 16.25f) &&
            equal(s.cylinder_status.lambda_coefficient, 17.25f) && equal(s.ignition_voltage, 18.25f) &&
            equal(s.throttle_out, 19.25f) && equal(s.pt_compensation, 20.25f);
    }
    const float *first = &s.spark_dwell_time_ms;
    const float *cylinder = &s.cylinder_status.ignition_timing_deg;
    for (unsigned i=0; i<10; i++) { if (!isnan(first[i])) { return false; } }
    for (unsigned i=0; i<7; i++) { if (!isnan(cylinder[i])) { return false; } }
    return s.last_updated_ms == 0 && uint8_t(s.engine_state) == 0 && !s.general_error &&
        s.engine_load_percent == 0 && s.engine_speed_rpm == 0 && s.throttle_position_percent == 0 &&
        s.ecu_index == 0 && uint8_t(s.spark_plug_usage) == 0 && s.ignition_voltage == -1 &&
        isnan(s.throttle_out) && isnan(s.pt_compensation);
}
extern "C" int get_backend(const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    return AP_Module::binding_efi_get_backend(in, ni, out, no);
}
extern "C" int handle(const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    return AP_Module::binding_efi_handle_scripting(in, ni, out, no);
}
extern "C" int get_state(const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    return AP_Module::binding_efi_get_state(in, ni, out, no);
}
extern "C" void configure(bool present, unsigned mode) { frontend_present=present; expected_mode=mode; }
''')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-DAP_MODULE_ENABLED=1', f'-DAP_EFI_SCRIPTING_ENABLED={int(request.param)}',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    for name in ('get_backend', 'handle', 'get_state'):
        getattr(lib, name).argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.configure.argtypes = [ctypes.c_bool, ctypes.c_uint]
    return lib, request.param


def test_backend_is_index_only_and_revalidated(efi):
    lib, enabled = efi
    output = ctypes.create_string_buffer(1)
    assert lib.get_backend(b'\0', 1, output, 1) == (0 if enabled else -3)
    assert output.raw == (b'\1' if enabled else b'\0')
    assert lib.get_backend(b'\1', 1, output, 1) == (0 if enabled else -3)
    assert output.raw == b'\0'
    lib.configure(False, 0)
    assert lib.handle(bytes(full_packet()), 102, output, 1) == -3
    assert output.raw == b'\0'


def test_complete_state_and_sentinels(efi):
    lib, enabled = efi
    output = ctypes.create_string_buffer(1)
    for mode, packet in enumerate((full_packet(), default_packet())):
        lib.configure(True, mode)
        assert lib.handle(bytes(packet), len(packet), output, 1) == (0 if enabled else -3)
        assert output.raw == (b'\1' if enabled else b'\0')
        state = ctypes.create_string_buffer(efi_binding.STATE_WIRE_SIZE)
        assert lib.get_state(None, 0, state, len(state)) == (0 if enabled else -3)
        assert state.raw == (bytes(packet[1:]) if enabled else bytes(efi_binding.STATE_WIRE_SIZE))


def test_invalid_wire_rejected(efi):
    lib, enabled = efi
    output = ctypes.create_string_buffer(1)
    for offset, value in [(5, 4), (6, 2), (7, 3), (8, 6), (9, 4), (10, 4),
                          (11, 3), (12, 3), (13, 3), (61, 4)]:
        packet = full_packet()
        packet[offset] = value
        assert lib.handle(bytes(packet), len(packet), output, 1) == (-1 if enabled else -3)
        assert output.raw == b'\0'
    packet = full_packet()
    for size in (100, 101, 103):
        assert lib.handle(bytes(packet), size, output, 1) == -1


def test_contract_and_sdk_are_owned(tmp_path):
    spec = efi_binding.select(bindings.read_ir(), 30)
    assert [method['id'] for method in spec['methods']] == [30, 31, 32]
    assert 'backend-index-revalidated-each-call' in spec['methods'][0]['policy']
    rust = efi_binding.rust(spec, '')
    assert '*mut' not in rust and '*const' not in rust
    assert 'pub struct EfiBackend { index: u8 }' in rust
    assert "impl crate::bindings::Efi<'_, '_>" in rust
    assert 'pub fn get_backend(&self' in rust
    assert 'pub fn get_state(&self' in rust
    assert 'pub fn efi_get_backend' not in rust
    assert 'pub fn efi_get_state' not in rust
    assert f'let mut input = [0u8; {efi_binding.STATE_WIRE_SIZE + 1}]' in rust
    source = tmp_path / 'efi.rs'
    source.write_text('''#![no_std]
pub enum Error { Invalid, Host }
pub type Result<T=()> = core::result::Result<T, Error>;
pub fn result(value: i32) -> Result { if value == 0 { Ok(()) } else { Err(Error::Host) } }
pub struct Api { pub binding_call: Option<unsafe extern "C" fn(
    *mut core::ffi::c_void, u32, *const u8, u32, *mut u8, u32) -> i32> }
pub struct Context<'a> { raw: *mut core::ffi::c_void, api: &'a Api }
pub mod bindings {
    use super::Context;
    pub struct Efi<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
pub mod imports {
    pub struct Import;
    impl Import { pub const fn slot(&self) -> u32 { 0 } }
    pub static BINDING_30: Import = Import;
    pub static BINDING_31: Import = Import;
    pub static BINDING_32: Import = Import;
}
mod generated {
''' + rust + '\n}\n')
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--crate-type', 'lib', '--edition', '2021',
                    str(source), '-o', str(tmp_path / 'efi.rlib')], check=True)


def test_ir_selection_fails_closed():
    ir = bindings.read_ir()
    assert efi_binding.select(ir, 30)['methods'][0]['name'] == 'efi.get_backend'

    mutations = []
    changed = copy.deepcopy(ir)
    next(item for item in changed['singletons'] if item['name'] == efi_binding.OWNER)['rename'] = 'efi2'
    mutations.append((changed, 'owner metadata'))
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['singletons'] if item['name'] == efi_binding.OWNER)
    next(method for method in owner['methods'] if method['name'] == 'get_backend')['arguments'][0]['skip_check'] = False
    mutations.append((changed, 'owner methods'))
    changed = copy.deepcopy(ir)
    backend = next(item for item in changed['ap_objects'] if item['name'] == efi_binding.BACKEND)
    backend['methods'][0]['arguments'][0]['reference'] = True
    mutations.append((changed, 'backend methods'))
    changed = copy.deepcopy(ir)
    state = next(item for item in changed['userdata'] if item['name'] == efi_binding.STATE)
    next(field for field in state['fields'] if field['name'] == 'ignition_voltage')['write'] = False
    mutations.append((changed, 'EFI state'))
    changed = copy.deepcopy(ir)
    cylinder = next(item for item in changed['userdata'] if item['name'] == efi_binding.CYLINDER)
    cylinder['fields'][0]['type']['skip_check'] = False
    mutations.append((changed, 'cylinder state'))

    for changed, error in mutations:
        with pytest.raises(ValueError, match=error):
            efi_binding.select(changed, 30)


def test_get_state_coverage_and_dispatch():
    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    state = coverage[(efi_binding.OWNER, 'get_state')]
    assert state['status'] == 'generated'
    assert state['reason'] == 'owned-efi-state-adapter'
    assert 'return binding_efi_get_state' in bindings.emit(spec)['Bindings_generated.cpp']

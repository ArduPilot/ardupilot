# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later

import copy
import ctypes
import json
import os
import struct
import subprocess

import bindings
import contracts
import generate
import pytest

# Observable stand-in subset only; production generation has no singleton list.
OBSERVABLE = {
    'AP_AHRS': dict(include='AP_AHRS/AP_AHRS.h', methods=[
        'get_roll_rad', 'get_pitch_rad', 'get_yaw_rad', 'get_EAS2TAS', 'airspeed_EAS', 'get_hagl',
        'get_gyro', 'get_velocity_NED', 'get_relative_position_NED_origin_float', 'get_quaternion']),
    'AP_Arming': dict(include='AP_Arming/AP_Arming.h', methods=['is_armed']),
    'AP_GPS': dict(include='AP_GPS/AP_GPS.h', methods=['num_sensors', 'primary_sensor']),
    'AP_Param': dict(include='AP_Param/AP_Param.h', methods=list(bindings.PARAM_ARGUMENTS)),
    'AP_Vehicle': dict(include='AP_Vehicle/AP_Vehicle.h', methods=['get_mode']),
    'AP::fwversion()': dict(include='AP_Common/AP_FWVersion.h', methods=[],
                            fields=['vehicle_type', 'major', 'minor', 'fw_short_string', 'fw_hash_str']),
    'RC_Channels': dict(include='RC_Channel/RC_Channel.h', methods=['get_pwm']),
}


def select(ir, selection):
    return bindings.select(ir, selection, only={name: selection[name] for name in OBSERVABLE})


@pytest.fixture(scope='module')
def ir():
    return bindings.read_ir()


@pytest.fixture(scope='module')
def selection():
    return dict(json.loads((bindings.ROOT / 'generator/bindings.json').read_text()), **copy.deepcopy(OBSERVABLE))


def test_contract_preserves_semantics(ir, selection):
    contract = select(ir, selection)
    methods = {method['rust_name']: method for method in contract['methods']}
    assert len(methods) == 27
    assert methods['ahrs_airspeed_eas']['aliases'] == ['ahrs_airspeed_estimate']
    assert methods['ahrs_get_hagl']['lock'] == 'semaphore'
    assert methods['vehicle_get_mode']['lock'] == 'scheduler-semaphore'
    assert methods['rc_get_pwm']['arguments'][0]['range'] == ['1', 'NUM_RC_CHANNELS']
    assert methods['param_add_table']['adapter'] == bindings.PARAM_POLICY
    assert methods['fwversion_type']['result']['type'] == 'uint8_t'
    assert methods['fwversion_string']['result']['type'] == 'fwstring'
    assert methods['fwversion_hash']['field'] == 'fw_hash_str'
    assert methods['fwversion_patch']['result']['type'] == 'uint8_t'
    reordered = dict(reversed(list(selection.items())))
    assert select(ir, reordered) == contract


@pytest.mark.parametrize('bad', ['owner', 'rename', 'kind', 'read', 'write', 'array', 'mask', 'missing'])
def test_fwversion_field_audit_fails_closed(ir, selection, bad):
    ir = copy.deepcopy(ir)
    owner = next(item for item in ir['singletons'] if item['name'] == 'AP::fwversion()')
    field = next(item for item in owner['fields'] if item['name'] == 'vehicle_type')
    if bad == 'owner':
        owner['reference'] = False
    elif bad == 'rename':
        field['rename'] = 'vehicle'
    elif bad == 'kind':
        field['type']['kind'] = 'uint16_t'
    elif bad == 'read':
        field['read'] = False
    elif bad == 'write':
        field['write'] = True
    elif bad == 'array':
        field['array_len'] = '2'
    elif bad == 'mask':
        field['valid_mask_name'] = 'flags'
    else:
        owner['fields'].remove(field)
    with pytest.raises(ValueError, match='firmware-version|accessor policy'):
        select(ir, selection)


@pytest.mark.parametrize('name', ['get_gyro', 'get_velocity_NED', 'get_relative_position_NED_origin_float', 'get_quaternion'])
@pytest.mark.parametrize('bad', ['pointer', 'reference', 'nullable', 'range', 'skip_check', 'name',
                                 'return'])
def test_ahrs_value_audit(ir, selection, name, bad):
    ir = copy.deepcopy(ir)
    owner = next(item for item in ir['singletons'] if item['name'] == 'AP_AHRS')
    method = next(item for item in owner['methods'] if item['name'] == name)
    value = method['arguments'][0] if method['arguments'] else method['return']
    if bad in ('pointer', 'reference', 'nullable', 'skip_check'):
        value[bad] = not value[bad]
    elif bad == 'range':
        value['range'] = ['0', '1']
    elif bad == 'name':
        value['name'] = 'UnknownValue'
    elif bad == 'return':
        method['return']['kind'] = 'float'
    if bad == 'nullable' and name != 'get_gyro':
        # Now a supported value input, with an explicitly different contract.
        contract = select(ir, selection)
        item = next(m for m in contract['methods'] if m['method'] == name)
        assert item['arguments'][0]['inout']
        assert not item['arguments'][0]['output']
        return
    with pytest.raises(ValueError):
        select(ir, selection)


@pytest.mark.parametrize('bad', ['reference', 'pointer', 'userdata', 'deprecated', 'dynamic_range', 'manual'])
def test_unsupported_selection_fails(ir, selection, bad):
    ir, selection = copy.deepcopy(ir), copy.deepcopy(selection)
    owner = next(owner for owner in ir['singletons'] if owner['name'] == 'AP_AHRS')
    method = next(method for method in owner['methods'] if method['name'] == 'get_hagl')
    if bad in ('reference', 'pointer'):
        method['arguments'][0][bad] = True
    elif bad == 'userdata':
        method['arguments'][0]['kind'] = 'userdata'
    elif bad == 'deprecated':
        method['deprecate'] = 'Use another method'
    elif bad == 'dynamic_range':
        method['arguments'][0]['range'] = ['0', 'ud->num_sensors()']
    elif bad == 'manual':
        selection['AP_AHRS']['methods'].append('wind_estimate')
    with pytest.raises(ValueError):
        select(ir, selection)


def test_fingerprint_contract(ir, selection):
    contract = select(ir, selection)
    spec = json.loads((bindings.ROOT / 'generator/host_api.json').read_text())
    original = generate.generate(spec, contract)['sdk/abi.json']
    # Guards are never evaluated using the generator host's configuration.
    guards = copy.deepcopy(contract)
    guards['methods'][0]['owner_depends'] = '0'
    assert generate.generate(spec, guards)['sdk/abi.json'] == original
    hashes = contracts.describe(contract)
    for field, value in [('lock', None), ('id', 99), ('method', 'different')]:
        changed = copy.deepcopy(contract)
        changed['methods'][0][field] = value
        assert generate.generate(spec, changed)['sdk/abi.json'] == original
        assert (contracts.describe(changed)[0]['hash'] != hashes[0]['hash']) == (field == 'lock')
    changed = copy.deepcopy(contract)
    next(m for m in changed['methods'] if m['rust_name'] == 'rc_get_pwm')['arguments'][0]['range'][1] = '8'
    assert generate.generate(spec, changed)['sdk/abi.json'] == original
    assert contracts.describe(changed) != hashes
    assert 'LOCK_TIMEOUT' not in generate.generate(spec, contract)['BindingConfig.h']
    changed = copy.deepcopy(contract)
    next(m for m in changed['methods'] if m['rust_name'] == 'param_add_table')['adapter'] += '-changed'
    assert generate.generate(spec, changed)['sdk/abi.json'] == original
    assert contracts.describe(changed) != hashes


@pytest.mark.parametrize('bad', ['pointer', 'skip_check', 'nullable', 'creation', 'fields', 'operator',
                                 'alias', 'init_by_info', 'return', 'missing'])
def test_parameter_audit_fails_closed(ir, selection, bad):
    ir, selection = copy.deepcopy(ir), copy.deepcopy(selection)
    owner = next(item for item in ir['userdata'] if item['name'] == 'Parameter')
    method = next(item for item in owner['methods'] if item['name'] == 'set')
    if bad in ('pointer', 'skip_check', 'nullable'):
        method['arguments'][0][bad] = not method['arguments'][0][bad]
    elif bad == 'creation':
        owner['creation_args'] = -1
    elif bad == 'fields':
        owner['has_fields'] = True
    elif bad == 'operator':
        owner['operations'] = 1
    elif bad == 'alias':
        owner['aliases'].append({'name': 'get', 'alias': 'custom', 'manual': True})
    elif bad == 'return':
        method['return']['kind'] = 'float'
    elif bad == 'missing':
        owner['methods'].remove(method)
    else:
        selection['Parameter']['methods'].append('init_by_info')
    with pytest.raises(ValueError):
        select(ir, selection)


def test_parameter_storage_affects_fingerprint(ir, selection):
    contract = select(ir, selection)
    spec = json.loads((bindings.ROOT / 'generator/host_api.json').read_text())
    original = generate.generate(spec, contract)['sdk/abi.json']
    hashes = contracts.describe(contract)
    contract['parameter']['policy'] += '-changed'
    assert generate.generate(spec, contract)['sdk/abi.json'] == original
    assert contracts.describe(contract) != hashes


def test_fwversion_public_contract_and_sdk(ir, selection):
    contract = select(ir, selection)
    entries = {entry['contract']['name']: entry['contract'] for entry in contracts.describe(contract)}
    string = entries['FWVersion.string']
    assert string['result'] == {
        'name': 'FirmwareString', 'length': 'u8', 'data': 'utf8-bytes-max64-zero-pad64',
    }
    assert 'reject-null-or-overlength' in string['policy']
    generated = bindings.emit(contract)
    rust = generated['sdk/rust/src/bindings.rs']
    assert 'pub fn fw_version(&self) -> FwVersion' in rust
    assert 'pub fn r#type(&self) -> Result<u8>' in rust
    assert 'pub fn string(&self) -> Result<FirmwareString>' in rust
    assert 'pub fn fwversion_type(' not in rust
    cpp = generated['Bindings_generated.cpp']
    assert 'const auto value = ud.fw_short_string;' in cpp
    assert 'strnlen(value, 65)' in cpp


def test_standalone_bindings_are_wired_into_public_generation():
    contract = bindings.specification()
    generated = bindings.emit(contract)
    cpp = generated['Bindings_generated.cpp']
    catalogue = {entry['contract']['name'] for entry in contracts.describe(contract)}

    expected_outputs = {
        'sdk/rust/src/applet.rs',
        'sdk/rust/src/crsf.rs',
        'sdk/rust/src/motor_config.rs',
        'sdk/rust/src/onvif.rs',
        'sdk/rust/src/serialled.rs',
        'sdk/rust/src/telemetry.rs',
        'sdk/rust/src/airspeed.rs',
        'sdk/rust/src/periph.rs',
        'sdk/rust/src/old_parameter.rs',
    }
    assert expected_outputs.issubset(generated)
    assert {
        'DroneCanHandle.broadcast',
        'esc_telem.update_telem_data',
        'servo_telem.get_telem',
        'servo_telem.update_telem_data',
        'airspeed.get_backend',
        'AirspeedBackend.handle_script_temperature',
        'periph.get_vehicle_state',
        'parameter.old_value',
        'MotorsDynamic.load_factors',
        'serialLED.set_num_neopixel',
        'onvif.start',
        'gps.inject_data',
        'camera.set_stream_information',
        'crsf.add_menu',
        'CRSFMenu.add_parameter',
    }.issubset(catalogue)
    for target in (
            'binding_dronecan_broadcast', 'binding_esc_telem_update_telem_data',
            'binding_motors_dynamic_load_factors', 'binding_serialled_set_num_neopixel',
            'binding_onvif_start', 'binding_applet_camera_set_stream_information'):
        assert target in cpp
    assert 'binding_crsf_menu_add_parameter' in cpp

    sdk = (bindings.ROOT / 'sdk/rust/src/lib.rs').read_text()
    for module in ('airspeed', 'applet', 'crsf', 'motor_config', 'old_parameter', 'onvif', 'periph',
                   'serialled', 'telemetry'):
        assert f'generated_module!({module});' in sdk
    for public_type in ('VideoStreamInformation', 'MotorFactorTable', 'EscTelemetryUpdate',
                        'ServoTelemetryData', 'ServoTelemetryUpdate', 'AirspeedBackend', 'OldParameterType',
                        'Ipv4Address', 'DroneCanHandle', 'CrsfMenuEvent'):
        assert public_type in sdk


@pytest.fixture(params=[True, False])
def dispatcher(tmp_path, ir, selection, request):
    """Compile the real generated dispatcher against observable C++ stand-ins."""
    contract = select(ir, selection)
    fixture = tmp_path / 'fixture.h'
    fixture.write_text('''#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
#define NUM_RC_CHANNELS 16
struct Vector3f { float x, y, z; };
struct Quaternion { float q1, q2, q3, q4; };
inline Vector3f gyro;
inline Quaternion attitude;
inline bool estimate_valid = true;
namespace AP_HAL {
class Semaphore {
public:
    bool available = true;
    unsigned depth = 0, calls = 0;
    bool take(uint32_t ms) { if (ms != 0) { __builtin_trap(); } calls++; depth++; return true; }
    void give() { if (depth != 1) { __builtin_trap(); } depth--; gyro = {}; }
};
}
inline AP_HAL::Semaphore ahrs_sem, scheduler_sem;
inline bool have_ahrs = true, have_scheduler = true;
inline unsigned method_calls = 0;
enum ap_var_type { AP_PARAM_INT8, AP_PARAM_INT16, AP_PARAM_INT32, AP_PARAM_FLOAT };
inline unsigned parameter_calls = 0;
inline float parameter_value = 0;
class AP_Param {
public:
    static AP_Param *get_singleton() { static AP_Param p; return &p; }
    static AP_Param *find(const char *name, ap_var_type *type) {
        if (!strcmp(name, "I8")) { *type = AP_PARAM_INT8; }
        else if (!strcmp(name, "I16")) { *type = AP_PARAM_INT16; }
        else if (!strcmp(name, "I32")) { *type = AP_PARAM_INT32; }
        else if (!strcmp(name, "F")) { *type = AP_PARAM_FLOAT; }
        else { return nullptr; }
        return get_singleton();
    }
    static bool get(const char *name, float &v) {
        ap_var_type type; parameter_calls++; v = 42; return find(name, &type) != nullptr;
    }
    static bool set_by_name(const char *name, float v) {
        ap_var_type type; parameter_calls++; parameter_value = v; return find(name, &type) != nullptr;
    }
    static bool set_and_save_by_name(const char *name, float v) { return set_by_name(name, v); }
    static bool set_default_by_name(const char *name, float v) { return set_by_name(name, v); }
    // No legacy add_table: selecting the wrong path must fail compilation.
    static bool add_table_persistent(uint8_t key, const char *name, uint8_t count) {
        parameter_calls++; return key == 200 && !strcmp(name, "TEST_") && count == 63;
    }
    static bool add_param(uint8_t key, uint8_t index, const char *name, float v) {
        parameter_calls++; parameter_value = v; return key == 200 && index == 63 && !strcmp(name, "VALUE");
    }
};
class AP_AHRS {
public:
    static AP_AHRS *get_singleton() { static AP_AHRS a; return have_ahrs ? &a : nullptr; }
    AP_HAL::Semaphore &get_semaphore() { return ahrs_sem; }
    float get_roll_rad() { if (ahrs_sem.depth != 1) { __builtin_trap(); } method_calls++; return -1.25f; }
    float get_pitch_rad() { return get_roll_rad(); }
    float get_yaw_rad() { return get_roll_rad(); }
    float get_EAS2TAS() { return get_roll_rad(); }
    bool get_hagl(float &v) { v = 999; get_roll_rad(); return false; }
    bool airspeed_EAS(float &v) { v = get_roll_rad(); return true; }
    const Vector3f &get_gyro() { get_roll_rad(); return gyro; }
    bool get_velocity_NED(Vector3f &v) { v = get_gyro(); return estimate_valid; }
    bool get_relative_position_NED_origin_float(Vector3f &v) { return get_velocity_NED(v); }
    bool get_quaternion(Quaternion &q) { get_roll_rad(); q = attitude; return estimate_valid; }
};
class AP_Arming {
public:
    static AP_Arming *get_singleton() { static AP_Arming a; return &a; }
    bool is_armed() { return true; }
};
class AP_GPS {
public:
    static AP_GPS *get_singleton() { static AP_GPS g; return &g; }
    uint8_t num_sensors() { return 2; }
    uint8_t primary_sensor() { return 1; }
};
struct BattMonitorScript_State {
    float voltage; bool healthy; uint8_t cell_count; uint8_t capacity_remaining_pct;
    uint8_t state_of_health_pct; uint16_t cell_voltages[32]; uint16_t cycle_count;
    float current_amps, consumed_mah, consumed_wh, temperature;
};
class AP_BattMonitor {
public:
    static AP_BattMonitor *get_singleton() { static AP_BattMonitor battery; return &battery; }
    bool handle_scripting(uint8_t, const BattMonitorScript_State &) { return false; }
};
class AP_Vehicle {
public:
    static AP_Vehicle *get_singleton() { static AP_Vehicle v; return &v; }
    uint8_t get_mode() { if (scheduler_sem.depth != 1) { __builtin_trap(); } method_calls++; return 10; }
};
class AP_FWVersion {
public:
    uint8_t vehicle_type = 3, major = 4, minor = 7, patch = 2;
    const char *fw_short_string = "ArduPlane V4.7.0";
    const char *fw_hash_str = "1234abcd";
};
namespace AP {
inline AP_FWVersion firmware_version;
inline const AP_FWVersion &fwversion() { return firmware_version; }
}
class RC_Channels {
public:
    static RC_Channels *get_singleton() { static RC_Channels r; return &r; }
    bool get_pwm(uint8_t c, uint16_t &v) {
        if (scheduler_sem.depth != 1 || c < 1 || c > 16) { __builtin_trap(); }
        method_calls++; v = 0x1234; return true;
    }
};
class AP_Scheduler {
public:
    AP_HAL::Semaphore &get_semaphore() { return scheduler_sem; }
    static AP_Scheduler *get_singleton() { static AP_Scheduler s; return have_scheduler ? &s : nullptr; }
};
''')
    parameter_header = tmp_path / 'AP_Scripting/Parameter.h'
    parameter_header.parent.mkdir(parents=True, exist_ok=True)
    parameter_header.write_text('''#pragma once
#include "fixture.h"
class alignas(64) Parameter {
public:
    bool init(const char *name) { vp = AP_Param::find(name, &vtype); return vp != nullptr; }
    ap_var_type type() const { return vtype; }
    bool get(float &value) { if (!vp) { return false; } value = parameter_value; return true; }
    bool set(float value) { if (!vp) { return false; } parameter_calls++; parameter_value = value; return true; }
    bool set_and_save(float value) { return set(value); }
    bool set_default(float value) { return set(value); }
    bool configured() { return vp != nullptr; }
private:
    ap_var_type vtype;
    AP_Param *vp;
    [[maybe_unused]] uint8_t padding[64];
};
''')
    paths = ['AP_HAL/AP_HAL_Boards.h', 'AP_HAL/Semaphores.h', 'AP_Scripting/AP_Scripting_config.h',
             'AP_BattMonitor/AP_BattMonitor.h', 'AP_BattMonitor/AP_BattMonitor_Scripting.h',
             'AP_Vehicle/AP_Vehicle_Type.h',
             'AP_Scheduler/AP_Scheduler.h'] + [value['include'] for value in selection.values()]
    for name in paths:
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('#include "fixture.h"\n')
    binding_ids = tmp_path / 'AP_Module/BindingIds.h'
    binding_ids.parent.mkdir(parents=True, exist_ok=True)
    binding_ids.write_text(contracts.emit(contract, '')['BindingIds.h'])
    wrapper = tmp_path / 'wrapper.cpp'
    wrapper.write_text('''#include "Bindings.h"
#include "BindingArming.h"
#include "BindingDroneCAN.h"
#include "BindingFile.h"
#include "BindingLog.h"
#include "BindingPrintf.h"
#include "BindingSharedState.h"
#include "fixture.h"
// Math cases have separate tests against real AP_Math, not stand-in math.
int32_t AP_Module::binding_math_dispatch(BindingId, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -1; }
// File IO has its own real-adapter fault-injection and filesystem tests.
int32_t AP_Module::binding_file_read_at(const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_log_write(const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
// Typed printf is covered by HAL and real-module integration tests.
int32_t AP_Module::binding_string_format_printf(const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
// Shared copies and lifecycle are exercised against the real service separately.
int32_t AP_Module::binding_shared_state_open(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_shared_state_read(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_shared_state_write(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_shared_state_close(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_dronecan_subscribe(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_dronecan_check_message(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_dronecan_close(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_dronecan_open(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_dronecan_broadcast(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_dronecan_request(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_arming_get_aux_auth_id(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_arming_set_aux_auth_passed(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
int32_t AP_Module::binding_arming_set_aux_auth_failed(void *, const uint8_t *, uint32_t, uint8_t *, uint32_t) { return -3; }
#pragma GCC visibility push(default)
extern "C" int32_t call(uint32_t id, const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no)
{ return AP_Module::binding_dispatch(nullptr, static_cast<AP_Module::BindingId>(id), in, ni, out, no); }
extern "C" void configure(bool present, bool available)
{ have_ahrs = present; ahrs_sem.available = scheduler_sem.available = available; }
extern "C" unsigned calls() { return method_calls; }
extern "C" unsigned held() { return ahrs_sem.depth + scheduler_sem.depth; }
extern "C" void scheduler_present(bool present) { have_scheduler = present; }
extern "C" unsigned param_calls() { return parameter_calls; }
extern "C" void firmware_strings(const char *short_string, const char *hash)
{ AP::firmware_version.fw_short_string = short_string; AP::firmware_version.fw_hash_str = hash; }
extern "C" void estimate(const void *bytes, bool valid) {
    memcpy(&gyro, bytes, sizeof(gyro)); memcpy(&attitude, bytes, sizeof(attitude)); estimate_valid = valid;
}
''')
    library = tmp_path / 'bindings.so'
    flags = ['AP_AHRS_ENABLED', 'AP_ARMING_ENABLED', 'AP_GPS_ENABLED',
             'AP_SCRIPTING_BINDING_VEHICLE_ENABLED', 'AP_RC_CHANNEL_ENABLED', 'AP_SCHEDULER_ENABLED',
             'AP_PARAM_DYNAMIC_ENABLED']
    generated = tmp_path / 'Bindings_generated.cpp'
    generated.write_text(bindings.emit(contract)['Bindings_generated.cpp'])
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-fPIC', '-fvisibility=hidden', '-shared', '-DAP_MODULE_ENABLED=1'] +
                   [f'-D{flag}={int(request.param)}' for flag in flags] +
                   ['-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(wrapper),
                    str(generated), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.call.restype = ctypes.c_int32
    lib.configure.argtypes = [ctypes.c_bool, ctypes.c_bool]
    lib.scheduler_present.argtypes = [ctypes.c_bool]
    lib.firmware_strings.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    lib.estimate.argtypes = [ctypes.c_void_p, ctypes.c_bool]
    ids = {method['rust_name']: method['id'] for method in contract['methods']}
    ids.update({'Parameter_' + method['name']: method['id'] for method in contract['parameter']['methods']})
    ids['Parameter_layout'] = contract['parameter']['layout']['id']
    return lib, ids, request.param


@pytest.mark.parametrize('name,size,nullable', [
    ('ahrs_get_gyro', 12, False),
    ('ahrs_get_velocity_ned', 13, True),
    ('ahrs_get_relative_position_ned_origin', 13, True),
    ('ahrs_get_quaternion', 17, True),
])
def test_ahrs_value_wire(dispatcher, name, size, nullable):
    lib, ids, enabled = dispatcher
    output = ctypes.create_string_buffer(b'x' * (size + 2), size + 2)
    pointer = ctypes.byref(output, 1)
    count = lib.calls()
    assert output.raw == b'x' * (size + 2) and lib.calls() == count
    # Exact component bits: negative zero, NaN payload, subnormal and infinity.
    sample = struct.pack('<4I', 0x80000000, 0x7fc12345, 1, 0x7f800000)
    for valid in (True, False):
        lib.estimate(sample, valid)
        assert lib.call(ids[name], None, 0, pointer, size) == (0 if enabled else -3)
        expected = ((b'\1' + sample[:size - 1]) if nullable else sample[:size])
        if nullable and (not enabled or not valid):
            expected = bytes(size)
        elif not enabled:
            expected = b'x' * size
        assert output.raw == b'x' + expected + b'x'
        assert lib.held() == 0
    for present, available, status in [(False, True, -3), (True, False, 0)]:
        lib.configure(present, available)
        count = lib.calls()
        output.raw = b'x' * (size + 2)
        expected_status = status if enabled else -3
        assert lib.call(ids[name], None, 0, pointer, size) == expected_status
        if expected_status == 0:
            assert lib.calls() == count + 1
        else:
            expected = (b'x' + bytes(size) + b'x') if nullable else b'x' * (size + 2)
            assert output.raw == expected
            assert lib.calls() == count
        assert lib.held() == 0


def test_cached_parameter_dispatch(dispatcher):
    lib, ids, _ = dispatcher
    layout = ctypes.create_string_buffer(b'x' * 10, 10)
    assert lib.call(ids['Parameter_layout'], None, 0, ctypes.byref(layout, 1), 8) == 0
    assert layout.raw == b'x' + struct.pack('<II', 128, 64) + b'x'
    size, alignment = struct.unpack('<II', layout.raw[1:9])
    storage = ctypes.create_string_buffer(b'x' * (size + 2 * alignment), size + 2 * alignment)
    start = ctypes.addressof(storage)
    address = (start + alignment) & ~(alignment - 1)
    output = ctypes.create_string_buffer(5)

    def call(name, extra=b'', pointer=address):
        data = struct.pack('<Q', pointer) + extra
        size = 5 if name == 'get' else 1
        return lib.call(ids['Parameter_' + name], data, len(data), output, size)

    def init(name):
        return call('init', name.encode().ljust(17, b'\0'))

    for method in ('init', 'get', 'set', 'set_and_save', 'set_default', 'configured'):
        ni = 25 if method == 'init' else (12 if method.startswith('set') else 8)
        no = 5 if method == 'get' else 1
        data = bytes(ni)
        count = lib.param_calls()
        # The packet is valid, but a null Parameter receiver is not.
        assert lib.call(ids['Parameter_' + method], data, ni, output, no) == -1
        assert lib.param_calls() == count
    assert call('init', b'F'.ljust(17, b'\0'), 0) == -1
    assert call('init', b'F'.ljust(17, b'\0'), address + 1) == -1
    assert call('init', b'F'.ljust(17, b'\0'), address + 8) == -1
    assert call('init', b'F' * 17) == -1
    assert call('init', b'F\0x'.ljust(17, b'\0')) == -1
    assert init('F') == 0 and output.raw[0] == 1

    offset = address - start
    assert storage.raw[:offset] == b'x' * offset
    assert storage.raw[offset + size:] == b'x' * (len(storage) - offset - size)
    assert call('set', struct.pack('<f', 73.0)) == 0 and output.raw[0] == 1
    assert call('get') == 0 and output.raw == struct.pack('<Bf', 1, 73.0)
    assert call('init', b'F\0x'.ljust(17, b'\0')) == -1
    assert call('get') == 0 and output.raw == struct.pack('<Bf', 1, 73.0)
    assert call('configured') == 0 and output.raw[0] == 1
    for method in ('set', 'set_and_save', 'set_default'):
        for name, low, high, outside in [('I8', -128.0, 127.0, 128.0),
                                         ('I16', -32768.0, 32767.0, 32768.0),
                                         ('I32', -2147483648.0, 2147483520.0, 2147483648.0)]:
            assert init(name) == 0
            rejected = [float('nan'), float('inf'), -float('inf'), outside, low * 2]
            if name != 'I32':
                rejected += [low - 0.5, high + 0.5]
            for value in rejected:
                count = lib.param_calls()
                assert call(method, struct.pack('<f', value)) == -1
                assert output.raw[0] == 0 and lib.param_calls() == count
            for value in (low, high, 1.5, -1.5):
                assert call(method, struct.pack('<f', value)) == 0 and output.raw[0] == 1
    assert init('MISSING') == 0 and output.raw[0] == 0
    assert call('get') == 0 and output.raw == b'\0' * 5
    assert call('set', struct.pack('<f', 1.0)) == 0 and output.raw[0] == 0
    assert call('configured') == 0 and output.raw[0] == 0
    assert init('F') == 0 and output.raw[0] == 1
    assert storage.raw[:offset] == b'x' * offset
    assert storage.raw[offset + size:] == b'x' * (len(storage) - offset - size)


def test_dispatch_values_and_locks(dispatcher):
    lib, ids, enabled = dispatcher
    output = ctypes.create_string_buffer(b'xxxxxx', 6)
    # Unaligned output is legal; zero input accepts nullptr.
    pointer = ctypes.byref(output, 1)
    assert lib.call(999, None, 0, pointer, 4) == -1
    status = lib.call(ids['ahrs_get_roll_rad'], None, 0, pointer, 4)
    if not enabled:
        assert status == -3
        assert output.raw == b'xxxxxx'
        assert lib.calls() == 0
        return
    assert status == 0
    assert output.raw == b'x\x00\x00\xa0\xbfx'
    assert lib.held() == 0
    assert lib.call(ids['ahrs_get_hagl'], None, 0, output, 5) == 0
    assert output.raw[:5] == bytes(5)  # false discards even a written C++ output
    assert lib.call(ids['ahrs_airspeed_eas'], None, 0, output, 5) == 0
    assert output.raw[:5] == b'\x01\x00\x00\xa0\xbf'
    before = lib.calls()
    for channel in (0, 17, 255):
        assert lib.call(ids['rc_get_pwm'], bytes([channel]), 1, output, 3) == -1
    assert lib.calls() == before and lib.held() == 0
    lib.configure(True, True)
    lib.scheduler_present(False)
    assert lib.call(ids['vehicle_get_mode'], None, 0, output, 1) == -3
    assert lib.call(ids['rc_get_pwm'], bytes([1]), 1, output, 3) == -3
    assert lib.held() == 0
    lib.scheduler_present(True)
    for channel in (1, 16):
        assert lib.call(ids['rc_get_pwm'], bytes([channel]), 1, output, 3) == 0
        assert output.raw[:3] == b'\x01\x34\x12'
    assert lib.call(ids['vehicle_get_mode'], None, 0, output, 1) == 0
    assert output.raw[0] == 10
    lib.configure(False, True)
    assert lib.call(ids['ahrs_get_roll_rad'], None, 0, pointer, 4) == -3
    lib.configure(True, False)
    assert lib.call(ids['ahrs_get_roll_rad'], None, 0, pointer, 4) == 0
    assert lib.call(ids['vehicle_get_mode'], None, 0, output, 1) == 0
    assert lib.call(ids['rc_get_pwm'], bytes([17]), 1, output, 3) == -1
    assert lib.held() == 0


def test_fwversion_field_wire(dispatcher):
    lib, ids, _ = dispatcher
    output = ctypes.create_string_buffer(67)
    for name, value in [('type', 3), ('major', 4), ('minor', 7), ('patch', 2)]:
        binding = ids['fwversion_' + name]
        assert lib.call(binding, None, 0, output, 1) == 0
        assert output.raw[0] == value

    for name, value in [('string', b'ArduPlane V4.7.0'), ('hash', b'1234abcd')]:
        binding = ids['fwversion_' + name]
        output.raw = b'x' * len(output)
        assert lib.call(binding, None, 0, ctypes.byref(output, 1), 65) == 0
        assert output.raw == b'x' + bytes([len(value)]) + value + bytes(64 - len(value)) + b'x'

    lib.firmware_strings(None, b'1234abcd')
    assert lib.call(ids['fwversion_string'], None, 0, output, 65) == -4
    assert output.raw[:65] == bytes(65)
    too_long = b'x' * 65
    lib.firmware_strings(too_long, b'1234abcd')
    assert lib.call(ids['fwversion_string'], None, 0, output, 65) == -4
    assert output.raw[:65] == bytes(65)


@pytest.mark.parametrize('kind', ['float', 'boolean', 'uint16_t', 'uint32_t'])
def test_scalar_input_categories(ir, selection, kind):
    ir = copy.deepcopy(ir)
    owner = next(owner for owner in ir['singletons'] if owner['name'] == 'RC_Channels')
    method = next(method for method in owner['methods'] if method['name'] == 'get_pwm')
    method['arguments'][0]['kind'] = kind
    contract = select(ir, selection)
    selected = next(m for m in contract['methods'] if m['rust_name'] == 'rc_get_pwm')
    assert selected['arguments'][0]['type'] == kind
    assert selected['arguments'][0]['range'] == ['1', 'NUM_RC_CHANNELS']


def test_parameter_wire_and_ranges(dispatcher):
    lib, ids, dynamic = dispatcher
    output = ctypes.create_string_buffer(5)

    def call(name, data, size=1):
        output.raw = b'xxxxx'
        return lib.call(ids['param_' + name], data, len(data), output, size)

    def name_field(name):
        return name.encode().ljust(17, b'\0')

    assert call('get', name_field('F'), 5) == 0
    assert output.raw == b'\1' + struct.pack('<f', 42)
    assert call('get', name_field('MISSING'), 5) == 0
    assert output.raw == bytes(5)
    # Fields are byte strings, not an invented restriction on parameter names.
    for name in ('a' * 16, 'lower_case', '\u00e9'):
        assert call('get', name_field(name), 5) == 0
    before = lib.param_calls()
    # The dispatcher trusts the contract's fixed wire size; exercise malformed
    # contents only, without asking it to read beyond a deliberately short packet.
    for data in (bytes(17), b'a' * 17, b'F\0junk'.ljust(17, b'\0')):
        assert call('get', data, 5) == -1
    assert lib.param_calls() == before
    for method in ('set', 'set_and_save', 'set_default'):
        for name, low, high, outside in [('I8', -128, 127, 128), ('I16', -32768, 32767, 32768),
                                         ('I32', -2147483648, 2147483520, 2147483648)]:
            for value in (low, high, 1.5, -1.5):
                assert call(method, name_field(name) + struct.pack('<f', value)) == 0
                assert output.raw[0] == 1
            for value in (outside, low * 2, float('nan'), float('inf'), -float('inf')):
                before = lib.param_calls()
                assert call(method, name_field(name) + struct.pack('<f', value)) == -1
                assert lib.param_calls() == before
        assert call(method, name_field('MISSING') + struct.pack('<f', 1)) == 0
        assert output.raw[0] == 0
        assert call(method, name_field('F') + struct.pack('<f', 3.4028234663852886e38)) == 0
        assert call(method, name_field('F') + struct.pack('<f', float('nan'))) == -1

    assert call('add_table', bytes([200]) + name_field('TEST_') + bytes([63])) == (0 if dynamic else -3)
    assert output.raw[0] == (1 if dynamic else ord('x'))
    assert call('add_param', bytes([200, 63]) + name_field('VALUE') + struct.pack('<f', 12)) == (0 if dynamic else -3)
    assert output.raw[0] == (1 if dynamic else ord('x'))
    if dynamic:
        before = lib.param_calls()
        for key, count in ((201, 1), (255, 1), (0, 0), (0, 64)):
            assert call('add_table', bytes([key]) + name_field('TEST_') + bytes([count])) == -1
            assert call('add_param', bytes([key, count]) + name_field('VALUE') + struct.pack('<f', 1)) == -1
        assert call('add_param', bytes([200, 1]) + name_field('VALUE') + struct.pack('<f', float('inf'))) == -1
        assert lib.param_calls() == before


@pytest.mark.parametrize('change', ['kind', 'key_range', 'count_range'])
def test_parameter_signature_change_requires_audit(ir, selection, change):
    ir = copy.deepcopy(ir)
    owner = next(owner for owner in ir['singletons'] if owner['name'] == 'AP_Param')
    method = next(method for method in owner['methods'] if method['name'] == 'add_table')
    if change == 'kind':
        method['arguments'][0]['kind'] = 'uint16_t'
    elif change == 'key_range':
        method['arguments'][0]['range'] = ['0', '255']
    else:
        method['arguments'][2]['range'] = ['0', '64']
    with pytest.raises(ValueError, match='unaudited-parameter-adapter'):
        select(ir, selection)


def test_manual_sdk_operations_are_namespaced():
    generated = bindings.emit(bindings.specification())
    for path, source in generated.items():
        if path.startswith('sdk/rust/src/') and path != 'sdk/rust/src/parameter.rs':
            assert "impl Context<'_>" not in source, path
    bindings_source = generated['sdk/rust/src/bindings.rs']
    for namespace in bindings.MANUAL_NAMESPACES:
        accessor = bindings.naming.rust_identifier(namespace)
        assert f'pub fn {accessor}(&self)' in bindings_source
    assert 'pub(crate) context:' in bindings_source

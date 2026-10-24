# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate bounded, non-rollback motor configuration bindings."""

import copy
import ctypes
import os
import struct
import subprocess

import bindings
import motor_config_binding
import pytest


@pytest.fixture(params=['copter', 'plane', 'disabled'])
def motor_host(tmp_path, request):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_Motors/AP_Motors_config.h': '''#pragma once
#define AP_MOTORS_FRAME_DYNAMIC_SCRIPTING_MATRIX_ENABLED TEST_MOTOR_BUILD
#define AP_MOTORS_FRAME_SCRIPTING_MATRIX_ENABLED TEST_MOTOR_BUILD
#define AP_MOTORS_FRAME_6DOF_SCRIPTING_ENABLED TEST_COPTER_BUILD
''',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t) { return true; } void give() {} }; }
using HAL_Semaphore = AP_HAL::Semaphore;
''',
        'AP_Scripting/AP_Scripting_config.h': '''#pragma once
#define AP_SCRIPTING_ENABLED 1
#define AP_SCRIPTING_BINDING_MOTORS_ENABLED 1
''',
        'AP_Vehicle/AP_Vehicle_Type.h': '''#pragma once
#define APM_BUILD_ArduPlane 1
#define APM_BUILD_ArduCopter 2
#define APM_BUILD_TYPE(type) ((type) == APM_BUILD_ArduPlane ? TEST_PLANE_BUILD : \
                              (type) == APM_BUILD_ArduCopter ? TEST_COPTER_BUILD : 0)
#define APM_BUILD_COPTER_OR_HELI TEST_COPTER_BUILD
''',
    }
    if request.param != 'disabled':
        headers.update({
            'AP_Math/AP_Math.h': '''#pragma once
inline bool is_zero(float value) { return value == 0.0f; }
''',
            'AP_Motors/AP_MotorsMatrix.h': '''#pragma once
#include <stdint.h>
#define AP_MOTORS_MAX_NUM_MOTORS 12
class AP_MotorsMatrix {
public:
    static AP_MotorsMatrix *get_singleton();
    bool init(uint8_t expected);
    void add_motor_raw(int8_t motor, float roll, float pitch, float yaw, uint8_t order);
    bool set_throttle_factor(int8_t motor, float factor);
};
''',
            'AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h': '''#pragma once
#include "AP_MotorsMatrix.h"
class AP_MotorsMatrix_Scripting_Dynamic {
public:
    struct factor_table {
        float roll[AP_MOTORS_MAX_NUM_MOTORS];
        float pitch[AP_MOTORS_MAX_NUM_MOTORS];
        float yaw[AP_MOTORS_MAX_NUM_MOTORS];
        float throttle[AP_MOTORS_MAX_NUM_MOTORS];
    };
    static AP_MotorsMatrix_Scripting_Dynamic *get_singleton();
    bool init(uint8_t expected);
    bool add_motor(uint8_t motor, uint8_t order);
    void load_factors(const factor_table &table);
};
''',
            'AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h': '''#pragma once
#include "AP_MotorsMatrix.h"
class AP_MotorsMatrix_6DoF_Scripting {
public:
    static AP_MotorsMatrix_6DoF_Scripting *get_singleton();
    bool init(uint8_t expected);
    void add_motor(int8_t motor, float roll, float pitch, float yaw, float throttle,
                   float forward, float right, bool reversible, uint8_t order);
};
''',
            'AP_Motors/AP_Motors_Class.h': '''#pragma once
class AP_Motors { public: void set_frame_string(const char *value); };
namespace AP { AP_Motors *motors(); }
''',
        })
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)

    source = tmp_path / 'motors.cpp'
    library = tmp_path / 'motors.so'
    source.write_text('''#include "BindingMotorConfig.h"
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#if TEST_MOTOR_BUILD
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.h>
#include <AP_Motors/AP_Motors_Class.h>
#if TEST_COPTER_BUILD
#include <AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h>
#endif
#endif
#include <string.h>
#if TEST_MOTOR_BUILD
static AP_MotorsMatrix matrix;
static AP_MotorsMatrix_Scripting_Dynamic dynamic;
static AP_Motors base;
static bool objects_present = true;
static uint8_t last_expected;
static uint8_t last_motor;
static uint8_t last_order;
static uint32_t last_factor_bits[4];
static uint32_t loaded_bits[4][AP_MOTORS_MAX_NUM_MOTORS];
static char frame[33];
#if TEST_COPTER_BUILD
static AP_MotorsMatrix_6DoF_Scripting sixdof;
static uint32_t sixdof_factor_bits[6];
static bool sixdof_reversible;
static uint32_t sixdof_calls;
#endif
static uint32_t bits(float value) { uint32_t result; memcpy(&result, &value, 4); return result; }
AP_MotorsMatrix *AP_MotorsMatrix::get_singleton() { return objects_present ? &matrix : nullptr; }
bool AP_MotorsMatrix::init(uint8_t expected) { last_expected = expected; return expected == 8; }
void AP_MotorsMatrix::add_motor_raw(int8_t motor, float roll, float pitch, float yaw, uint8_t order) {
    last_motor = uint8_t(motor); last_order = order;
    last_factor_bits[0] = bits(roll); last_factor_bits[1] = bits(pitch); last_factor_bits[2] = bits(yaw);
}
bool AP_MotorsMatrix::set_throttle_factor(int8_t motor, float factor) {
    last_motor = uint8_t(motor); last_factor_bits[3] = bits(factor); return factor <= 1.0f;
}
AP_MotorsMatrix_Scripting_Dynamic *AP_MotorsMatrix_Scripting_Dynamic::get_singleton() {
    return objects_present ? &dynamic : nullptr;
}
bool AP_MotorsMatrix_Scripting_Dynamic::init(uint8_t expected) {
    last_expected = expected; return expected == 8;
}
bool AP_MotorsMatrix_Scripting_Dynamic::add_motor(uint8_t motor, uint8_t order) {
    last_motor = motor; last_order = order; return motor != 7;
}
void AP_MotorsMatrix_Scripting_Dynamic::load_factors(const factor_table &table) {
    for (uint8_t index = 0; index < AP_MOTORS_MAX_NUM_MOTORS; index++) {
        loaded_bits[0][index] = bits(table.roll[index]);
        loaded_bits[1][index] = bits(table.pitch[index]);
        loaded_bits[2][index] = bits(table.yaw[index]);
        loaded_bits[3][index] = bits(table.throttle[index]);
    }
}
#if TEST_COPTER_BUILD
AP_MotorsMatrix_6DoF_Scripting *AP_MotorsMatrix_6DoF_Scripting::get_singleton() {
    return objects_present ? &sixdof : nullptr;
}
bool AP_MotorsMatrix_6DoF_Scripting::init(uint8_t expected) {
    last_expected = expected; return expected == 6;
}
void AP_MotorsMatrix_6DoF_Scripting::add_motor(int8_t motor, float roll, float pitch, float yaw,
                                                float throttle, float forward, float right,
                                                bool reversible, uint8_t order) {
    last_motor = uint8_t(motor); last_order = order;
    const float factors[] = { roll, pitch, yaw, throttle, forward, right };
    for (uint8_t index = 0; index < 6; index++) { sixdof_factor_bits[index] = bits(factors[index]); }
    sixdof_reversible = reversible;
    sixdof_calls++;
}
#endif
namespace AP { AP_Motors *motors() { return objects_present ? &base : nullptr; } }
void AP_Motors::set_frame_string(const char *value) {
    if (frame[0] == 0) { strncpy(frame, value, sizeof(frame) - 1); }
}
#endif
extern "C" int call(unsigned id, const uint8_t *input, uint32_t input_size,
                    uint8_t *output, uint32_t output_size) {
    switch (id) {
    case 0: return AP_Module::binding_motors_dynamic_init(input, input_size, output, output_size);
    case 1: return AP_Module::binding_motors_dynamic_add_motor(input, input_size, output, output_size);
    case 2: return AP_Module::binding_motors_dynamic_load_factors(input, input_size, output, output_size);
    case 3: return AP_Module::binding_motors_matrix_init(input, input_size, output, output_size);
    case 4: return AP_Module::binding_motors_matrix_add_motor_raw(input, input_size, output, output_size);
    case 5: return AP_Module::binding_motors_matrix_set_throttle_factor(input, input_size, output, output_size);
    case 6: return AP_Module::binding_motors_set_frame_string(input, input_size, output, output_size);
    case 7: return AP_Module::binding_motors_6dof_init(input, input_size, output, output_size);
    case 8: return AP_Module::binding_motors_6dof_add_motor(input, input_size, output, output_size);
    default: return -1;
    }
}
extern "C" void configure(bool present) {
#if TEST_MOTOR_BUILD
    objects_present = present;
#else
    (void)present;
#endif
}
extern "C" uint8_t expected() {
#if TEST_MOTOR_BUILD
    return last_expected;
#else
    return 0;
#endif
}
extern "C" uint8_t motor() {
#if TEST_MOTOR_BUILD
    return last_motor;
#else
    return 0;
#endif
}
extern "C" uint8_t order() {
#if TEST_MOTOR_BUILD
    return last_order;
#else
    return 0;
#endif
}
extern "C" uint32_t factor(unsigned field) {
#if TEST_MOTOR_BUILD
    return last_factor_bits[field];
#else
    (void)field; return 0;
#endif
}
extern "C" uint32_t loaded(unsigned field, unsigned index) {
#if TEST_MOTOR_BUILD
    return loaded_bits[field][index];
#else
    (void)field; (void)index; return 0;
#endif
}
extern "C" const char *frame_string() {
#if TEST_MOTOR_BUILD
    return frame;
#else
    return "";
#endif
}
extern "C" uint32_t sixdof_factor(unsigned index) {
#if TEST_COPTER_BUILD
    return sixdof_factor_bits[index];
#else
    (void)index; return 0;
#endif
}
extern "C" bool sixdof_is_reversible() {
#if TEST_COPTER_BUILD
    return sixdof_reversible;
#else
    return false;
#endif
}
extern "C" uint32_t sixdof_call_count() {
#if TEST_COPTER_BUILD
    return sixdof_calls;
#else
    return 0;
#endif
}
''')
    motor_enabled = int(request.param != 'disabled')
    copter_enabled = int(request.param == 'copter')
    plane_enabled = int(request.param == 'plane')
    subprocess.run([
        os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
        '-DAP_MODULE_ENABLED=1', f'-DTEST_MOTOR_BUILD={motor_enabled}',
        f'-DTEST_COPTER_BUILD={copter_enabled}', f'-DTEST_PLANE_BUILD={plane_enabled}',
        '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source),
        str(bindings.ROOT / 'BindingMotorConfig.cpp'), '-o', str(library),
    ], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_uint, ctypes.c_void_p, ctypes.c_uint32,
                         ctypes.c_void_p, ctypes.c_uint32]
    lib.call.restype = ctypes.c_int
    lib.configure.argtypes = [ctypes.c_bool]
    for name in ('expected', 'motor', 'order'):
        getattr(lib, name).restype = ctypes.c_uint8
    lib.factor.argtypes = [ctypes.c_uint]
    lib.factor.restype = ctypes.c_uint32
    lib.loaded.argtypes = [ctypes.c_uint, ctypes.c_uint]
    lib.loaded.restype = ctypes.c_uint32
    lib.frame_string.restype = ctypes.c_char_p
    lib.sixdof_factor.argtypes = [ctypes.c_uint]
    lib.sixdof_factor.restype = ctypes.c_uint32
    lib.sixdof_is_reversible.restype = ctypes.c_bool
    lib.sixdof_call_count.restype = ctypes.c_uint32
    return lib, request.param


def bool_call(lib, binding_id, packet):
    output = ctypes.create_string_buffer(1)
    status = lib.call(binding_id, packet, len(packet), output, 1)
    return status, output.raw[0]


def test_singletons_are_revalidated_and_native_boole_preserved(motor_host):
    lib, mode = motor_host
    enabled = mode != 'disabled'
    lib.configure(True)
    expected = 0 if enabled else -3
    assert bool_call(lib, 0, b'\x08') == (expected, 1 if enabled else 0)
    assert bool_call(lib, 1, b'\x07\x08') == (expected, 0)
    assert bool_call(lib, 1, b'\x06\x08') == (expected, 1 if enabled else 0)
    assert bool_call(lib, 3, b'\x08') == (expected, 1 if enabled else 0)
    assert bool_call(lib, 5, b'\x04' + struct.pack('<f', 1.25)) == (expected, 0)
    if enabled:
        assert lib.expected() == 8
        assert (lib.motor(), lib.order()) == (4, 8)
    lib.configure(False)
    assert bool_call(lib, 0, b'\x08') == (-3, 0)
    assert bool_call(lib, 5, b'\x04' + struct.pack('<f', 1.0)) == (-3, 0)


def test_factor_table_is_complete_finite_and_target_bounded(motor_host):
    lib, mode = motor_host
    enabled = mode != 'disabled'
    lib.configure(True)
    fields = [[field * 100.0 + index for index in range(12)] + [0.0] * 20 for field in range(4)]
    packet = b''.join(struct.pack('<f', value) for field in fields for value in field)
    expected = 0 if enabled else -3
    assert lib.call(2, packet, len(packet), None, 0) == expected
    if enabled:
        for field in range(4):
            for index in (0, 5, 11):
                assert lib.loaded(field, index) == struct.unpack('<I', struct.pack('<f', fields[field][index]))[0]

    noncanonical = bytearray(packet)
    noncanonical[12 * 4:13 * 4] = struct.pack('<f', 0.5)
    invalid = -1 if enabled else -3
    assert lib.call(2, bytes(noncanonical), len(noncanonical), None, 0) == invalid
    nonfinite = bytearray(packet)
    nonfinite[4:8] = struct.pack('<I', 0x7FC12345)
    assert lib.call(2, bytes(nonfinite), len(nonfinite), None, 0) == invalid


def test_static_matrix_validates_indices_and_factors(motor_host):
    lib, mode = motor_host
    enabled = mode != 'disabled'
    lib.configure(True)
    packet = b'\x0b' + struct.pack('<fff', 0.25, -0.5, 1.0) + b'\x0c'
    expected = 0 if enabled else -3
    assert lib.call(4, packet, len(packet), None, 0) == expected
    if enabled:
        assert (lib.motor(), lib.order()) == (11, 12)
        assert lib.factor(0) == struct.unpack('<I', struct.pack('<f', 0.25))[0]

    invalid = -1 if enabled else -3
    assert lib.call(4, b'\x0c' + packet[1:], len(packet), None, 0) == invalid
    assert lib.call(4, packet[:-1] + b'\x0d', len(packet), None, 0) == invalid
    nan_packet = b'\0' + struct.pack('<Iff', 0x7FC00000, 0.0, 0.0) + b'\0'
    assert lib.call(4, nan_packet, len(nan_packet), None, 0) == invalid
    assert bool_call(lib, 5, b'\0' + struct.pack('<f', -0.1)) == (invalid, 0)
    assert bool_call(lib, 5, b'\0' + struct.pack('<I', 0x7F800000)) == (invalid, 0)


def test_6dof_is_copter_only_complete_finite_and_persistent(motor_host):
    lib, mode = motor_host
    lib.configure(True)
    copter = mode == 'copter'
    expected = 0 if copter else -3
    assert bool_call(lib, 7, b'\x06') == (expected, 1 if copter else 0)

    factors = [0.25, -0.5, 1.0, 0.75, -1.0, 0.125]
    packet = b'\x0b' + b''.join(struct.pack('<f', value) for value in factors) + b'\x01\x0c'
    assert len(packet) == 27
    assert lib.call(8, packet, len(packet), None, 0) == expected
    if copter:
        assert (lib.motor(), lib.order()) == (11, 12)
        assert lib.sixdof_is_reversible()
        assert lib.sixdof_call_count() == 1
        for index, value in enumerate(factors):
            assert lib.sixdof_factor(index) == struct.unpack('<I', struct.pack('<f', value))[0]

    invalid = -1 if copter else -3
    malformed_packets = [
        b'\x0c' + packet[1:],
        packet[:-2] + b'\x02' + packet[-1:],
        packet[:-1] + b'\x0d',
    ]
    for index in range(6):
        nonfinite = bytearray(packet)
        nonfinite[1 + index * 4:5 + index * 4] = struct.pack('<I', 0x7FC00000 + index)
        malformed_packets.append(bytes(nonfinite))
    for malformed in malformed_packets:
        assert lib.call(8, malformed, len(malformed), None, 0) == invalid
    if copter:
        assert lib.sixdof_call_count() == 1

    lib.configure(False)
    assert bool_call(lib, 7, b'\x06') == (-3, 0)
    assert lib.call(8, packet, len(packet), None, 0) == -3


def test_frame_string_is_copied_bounded_and_first_write_wins(motor_host):
    lib, mode = motor_host
    enabled = mode != 'disabled'
    lib.configure(True)

    def frame_packet(value):
        return bytes([len(value)]) + value + bytes(motor_config_binding.FRAME_STRING_MAX - len(value))

    label = b'Dynamic CoM adjust'
    packet = frame_packet(label)
    expected = 0 if enabled else -3
    assert lib.call(6, packet, len(packet), None, 0) == expected
    if enabled:
        assert lib.frame_string() == label
    assert lib.call(6, frame_packet(b'replacement'), 33, None, 0) == expected
    if enabled:
        assert lib.frame_string() == label

    embedded_nul = frame_packet(b'a\0b')
    invalid = -1 if enabled else -3
    assert lib.call(6, embedded_nul, len(embedded_nul), None, 0) == invalid
    noncanonical = bytearray(frame_packet(b'abc'))
    noncanonical[-1] = 1
    assert lib.call(6, bytes(noncanonical), len(noncanonical), None, 0) == invalid
    assert lib.call(6, bytes([33]) + bytes(32), 33, None, 0) == invalid


def test_wire_shapes_are_strict(motor_host):
    lib, _ = motor_host
    output = ctypes.create_string_buffer(1)
    assert lib.call(0, b'', 0, output, 1) == -1
    assert lib.call(0, b'\0', 1, None, 1) == -1
    assert lib.call(2, bytes(511), 511, None, 0) == -1
    assert lib.call(4, bytes(14), 14, output, 1) == -1
    assert lib.call(6, bytes(33), 33, output, 1) == -1
    assert lib.call(7, b'', 0, output, 1) == -1
    assert lib.call(8, bytes(26), 26, None, 0) == -1
    assert lib.call(8, bytes(27), 27, output, 1) == -1


def test_contract_and_generated_rust_compile(tmp_path):
    spec = motor_config_binding.select(bindings.read_ir(), 70)
    assert [method['id'] for method in spec['methods']] == list(range(70, 79))
    assert all('no-rollback-on-module-exit' in method['policy'] for method in spec['methods'])
    assert all('copter-only' in method['policy'] for method in spec['methods'][-2:])
    assert spec['factor_count'] == 32 and spec['frame_string_max'] == 32
    rust = motor_config_binding.rust(spec, '')
    assert 'pub struct MotorFactorTable' in rust
    assert 'value.is_finite()' in rust
    assert 'Changes persist after this module exits' in rust
    assert "impl crate::bindings::Motors6dof<'_, '_>" in rust
    assert 'pub fn init(&self, expected_motors: u8)' in rust
    assert 'pub fn add_motor(&self, motor: u8' in rust
    source = tmp_path / 'motor_config.rs'
    imports = ''.join(f'pub static BINDING_{index}: Import = Import({index});\n' for index in range(70, 79))
    source.write_text('''use core::ffi::c_void;
#[derive(Clone, Copy, Debug, Eq, PartialEq)] pub enum Error { Invalid, Host }
pub type Result<T=()> = core::result::Result<T, Error>;
pub fn result(value: i32) -> Result { if value == 0 { Ok(()) } else { Err(Error::Host) } }
pub struct Api { pub binding_call: Option<unsafe extern "C" fn(
    *mut c_void, u32, *const u8, u32, *mut u8, u32) -> i32> }
pub struct Context<'a> { raw: *mut c_void, api: &'a Api }
pub mod bindings {
    use super::Context;
    pub struct MotorsDynamic<'context, 'api> { pub(crate) context: &'context Context<'api> }
    pub struct MotorsMatrix<'context, 'api> { pub(crate) context: &'context Context<'api> }
    pub struct Motors6dof<'context, 'api> { pub(crate) context: &'context Context<'api> }
    pub struct Motors<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
impl<'api> Context<'api> {
    pub fn motors_dynamic(&self) -> bindings::MotorsDynamic<'_, 'api> {
        bindings::MotorsDynamic { context: self }
    }
    pub fn motors_matrix(&self) -> bindings::MotorsMatrix<'_, 'api> {
        bindings::MotorsMatrix { context: self }
    }
    pub fn motors_6dof(&self) -> bindings::Motors6dof<'_, 'api> {
        bindings::Motors6dof { context: self }
    }
    pub fn motors(&self) -> bindings::Motors<'_, 'api> { bindings::Motors { context: self } }
}
pub mod imports { pub struct Import(pub u32); impl Import { pub const fn slot(&self) -> u32 { self.0 } }
''' + imports + '}\nmod generated {\n' + rust + r'''
}
#[cfg(test)]
mod tests {
    use super::{generated::*, Api, Context, Error};
    use core::ffi::c_void;

    struct Host { id: u32, input: [u8; 27], input_size: usize, calls: u32 }
    unsafe extern "C" fn call(raw: *mut c_void, id: u32, input: *const u8, input_size: u32,
                               output: *mut u8, output_size: u32) -> i32 {
        let host = &mut *raw.cast::<Host>();
        host.id = id;
        host.calls += 1;
        host.input_size = input_size as usize;
        host.input[..host.input_size].copy_from_slice(
            core::slice::from_raw_parts(input, host.input_size));
        if output_size == 1 { *output = 1; }
        0
    }

    #[test]
    fn six_dof_exact_wire_and_pre_host_validation() {
        let mut host = Host { id: 0, input: [0; 27], input_size: 0, calls: 0 };
        let api = Api { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        assert_eq!(context.motors_6dof().init(6), Ok(true));
        assert_eq!((host.id, host.input_size), (77, 1));
        assert_eq!(host.input[0], 6);

        let factors = [0.25f32, -0.5, 1.0, 0.75, -1.0, 0.125];
        assert_eq!(context.motors_6dof().add_motor(
            11, factors[0], factors[1], factors[2], factors[3], factors[4], factors[5], true, 12), Ok(()));
        assert_eq!((host.id, host.input_size), (78, 27));
        assert_eq!(host.input[0], 11);
        for (index, value) in factors.iter().enumerate() {
            assert_eq!(&host.input[1 + index * 4..5 + index * 4], &value.to_le_bytes());
        }
        assert_eq!(&host.input[25..], &[1, 12]);

        let calls = host.calls;
        assert_eq!(context.motors_6dof().add_motor(
            0, 0.0, 0.0, 0.0, f32::NAN, 0.0, 0.0, false, 1), Err(Error::Invalid));
        assert_eq!(host.calls, calls);
    }
}
''')
    binary = tmp_path / 'motor_config_test'
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--test', '--edition', '2021',
                    str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


def test_ir_selection_fails_closed():
    ir = bindings.read_ir()
    assert motor_config_binding.select(ir, 70)['methods'][0]['name'] == 'MotorsDynamic.init'
    mutations = []
    changed = copy.deepcopy(ir)
    dynamic = next(item for item in changed['singletons'] if item['name'] == motor_config_binding.DYNAMIC_OWNER)
    next(method for method in dynamic['methods'] if method['name'] == 'add_motor')['return']['kind'] = 'boolean'
    mutations.append((changed, 'dynamic motor matrix'))
    changed = copy.deepcopy(ir)
    matrix = next(item for item in changed['singletons'] if item['name'] == motor_config_binding.MATRIX_OWNER)
    next(method for method in matrix['methods'] if method['name'] == 'add_motor_raw')['arguments'][1]['skip_check'] = False
    mutations.append((changed, 'static motor matrix methods'))
    changed = copy.deepcopy(ir)
    table = next(item for item in changed['userdata'] if item['name'] == motor_config_binding.FACTOR_TABLE)
    next(field for field in table['fields'] if field['name'] == 'roll')['write'] = False
    mutations.append((changed, 'motor factor table fields'))
    changed = copy.deepcopy(ir)
    motors = next(item for item in changed['singletons'] if item['name'] == motor_config_binding.MOTORS_OWNER)
    next(method for method in motors['methods'] if method['name'] == 'set_frame_string')['arguments'][0]['kind'] = 'uint8_t'
    mutations.append((changed, 'motors frame string'))
    changed = copy.deepcopy(ir)
    six_dof = next(item for item in changed['singletons'] if item['name'] == motor_config_binding.SIX_DOF_OWNER)
    next(method for method in six_dof['methods'] if method['name'] == 'add_motor')['arguments'][4]['skip_check'] = False
    mutations.append((changed, '6DoF motor matrix'))
    for changed, message in mutations:
        with pytest.raises(ValueError, match=message):
            motor_config_binding.select(changed, 70)

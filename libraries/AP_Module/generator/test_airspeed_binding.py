# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise revalidated scripting-air-speed backends."""

import copy
import ctypes
import os
import struct
import subprocess

import airspeed_binding
import bindings
import pytest


def test_selection_is_fail_closed():
    ir = bindings.read_ir()
    spec = airspeed_binding.select(ir, 700)
    assert [method['id'] for method in spec['methods']] == list(range(700, 704))
    changed = copy.deepcopy(ir)
    backend = next(item for item in changed['ap_objects'] if item['name'] == airspeed_binding.BACKEND)
    next(method for method in backend['methods']
         if method['name'] == 'handle_script_temperature')['arguments'][0]['skip_check'] = False
    with pytest.raises(ValueError, match='handle_script_temperature'):
        airspeed_binding.select(changed, 700)


@pytest.mark.parametrize('enabled', [True, False])
def test_host_revalidates_and_forwards_all_float_bits(tmp_path, enabled):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '#pragma once\n#define AP_MODULE_ENABLED 1\n',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { class Semaphore { public: bool take(uint32_t) { return true; } void give() {} }; }
''',
        'AP_Airspeed/AP_Airspeed.h': '''#pragma once
#include <stdint.h>
#define APM_BUILD_ArduPlane 1
#define APM_BUILD_TYPE(x) (x)
class AP_Airspeed_Backend;
class AP_Airspeed { public: static AP_Airspeed *get_singleton(); AP_Airspeed_Backend *get_backend(uint8_t); };
''',
        'AP_Airspeed/AP_Airspeed_Backend.h': '''#pragma once
class AP_Airspeed_Backend { public:
    bool handle_script_airspeed(float); bool handle_script_differential_pressure(float);
    bool handle_script_temperature(float);
};
''',
        'AP_Vehicle/AP_Vehicle_Type.h': '#pragma once\n',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source = tmp_path / 'airspeed.cpp'
    source.write_text(r'''
#include "BindingAirspeed.h"
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Airspeed/AP_Airspeed_Backend.h>
#include <string.h>
static AP_Airspeed frontend; static AP_Airspeed_Backend backend;
static bool frontend_present=true, backend_present=true; static uint8_t operation; static uint32_t value_bits;
AP_Airspeed *AP_Airspeed::get_singleton() { return frontend_present ? &frontend : nullptr; }
AP_Airspeed_Backend *AP_Airspeed::get_backend(uint8_t index) {
    return backend_present && index == 2 ? &backend : nullptr;
}
static void seen(uint8_t op, float value) { operation=op; memcpy(&value_bits,&value,4); }
bool AP_Airspeed_Backend::handle_script_airspeed(float value) { seen(1,value); return true; }
bool AP_Airspeed_Backend::handle_script_differential_pressure(float value) { seen(2,value); return true; }
bool AP_Airspeed_Backend::handle_script_temperature(float value) { seen(3,value); return true; }
extern "C" int call(uint8_t method,const uint8_t *i,uint32_t n,uint8_t *o,uint32_t z) {
    if (method == 0) return AP_Module::binding_airspeed_get_backend(i,n,o,z);
    return AP_Module::binding_airspeed_script_value(i,n,o,z,method-1);
}
extern "C" void configure(bool fp,bool bp) { frontend_present=fp; backend_present=bp; }
extern "C" uint32_t observed() { return (uint32_t(operation)<<24) | (value_bits & 0xFFFFFF); }
extern "C" uint32_t bits() { return value_bits; }
''')
    library = tmp_path / 'airspeed.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-shared', '-fPIC', f'-DAP_AIRSPEED_SCRIPTING_ENABLED={int(enabled)}',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source),
                    str(bindings.ROOT / 'BindingAirspeed.cpp'), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_uint8, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.configure.argtypes = [ctypes.c_bool, ctypes.c_bool]
    lib.bits.restype = ctypes.c_uint32
    output = ctypes.create_string_buffer(1)
    expected = 0 if enabled else -3
    assert lib.call(0, b'\2', 1, output, 1) == expected
    assert output.raw[0] == (1 if enabled else 0)
    value = 0x7FC12345
    for method, operation in ((1, 1), (2, 2), (3, 3)):
        packet = b'\2' + struct.pack('<I', value)
        assert lib.call(method, packet, 5, output, 1) == expected
        if enabled:
            assert output.raw[0] == 1 and lib.bits() == value and lib.observed() >> 24 == operation
    if enabled:
        lib.configure(True, False)
        assert lib.call(1, b'\2' + bytes(4), 5, output, 1) == -3


def test_emitted_rust_compiles(tmp_path):
    rust = airspeed_binding.rust(airspeed_binding.specification(700), '')
    assert "impl crate::bindings::Airspeed<'_, '_>" in rust
    assert 'pub fn get_backend(&self, index: u8)' in rust
    source = tmp_path / 'airspeed.rs'
    imports = '\n'.join(f'pub static BINDING_{value}: Import=Import;' for value in range(700, 704))
    source.write_text('''#![no_std]
pub enum Error { Host } pub type Result<T=()> = core::result::Result<T,Error>;
pub fn result(v:i32)->Result { if v==0 {Ok(())} else {Err(Error::Host)} }
pub struct Api { pub binding_call: Option<unsafe extern "C" fn(*mut core::ffi::c_void,u32,*const u8,u32,*mut u8,u32)->i32> }
pub struct Context<'a> { raw:*mut core::ffi::c_void, api:&'a Api }
pub mod bindings {
    use super::Context;
    pub struct Airspeed<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
pub mod imports { pub struct Import; impl Import { pub const fn slot(&self)->u32 {0} }
''' + imports + '}\nmod generated {\n' + rust + '}\n')
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--crate-type', 'lib', '--edition=2021',
                    str(source), '-o', str(tmp_path / 'airspeed.rlib')], check=True)

# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise leak-free old-parameter value lookups."""

import ctypes
import os
import struct
import subprocess

import bindings
import old_parameter_binding


def test_selection_and_emitted_rust(tmp_path):
    spec = old_parameter_binding.select(bindings.read_ir(), 950)
    assert spec['methods'][0]['name'] == 'parameter.old_value'
    generated = old_parameter_binding.rust(spec, '')
    assert "impl crate::bindings::Param<'_, '_>" in generated
    assert 'pub fn old_value(' in generated
    source = tmp_path / 'old.rs'
    source.write_text('''#![no_std]
pub enum Error {Host} pub type Result<T=()>=core::result::Result<T,Error>;
pub fn result(v:i32)->Result{if v==0{Ok(())}else{Err(Error::Host)}}
pub struct Api{pub binding_call:Option<unsafe extern "C" fn(*mut core::ffi::c_void,u32,*const u8,u32,*mut u8,u32)->i32>}
pub struct Context<'a>{raw:*mut core::ffi::c_void,api:&'a Api}
pub mod bindings {
    use super::Context;
    pub struct Param<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
pub mod imports{pub struct Import;impl Import{pub const fn slot(&self)->u32{0}}pub static BINDING_950:Import=Import;}
mod generated {
''' + generated + '}\n')
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--crate-type', 'lib', '--edition=2021',
                    str(source), '-o', str(tmp_path / 'old.rlib')], check=True)


def test_host_uses_stack_typed_value(tmp_path):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '#pragma once\n#define AP_MODULE_ENABLED 1\n',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &)=delete; C &operator=(const C &)=delete
namespace AP_HAL { class Semaphore { public: bool take(uint32_t){return true;} void give(){} }; }
''',
        'AP_Param/AP_Param.h': '''#pragma once
#include <stdint.h>
enum ap_var_type { AP_PARAM_NONE=0, AP_PARAM_INT8=1, AP_PARAM_INT16=2, AP_PARAM_INT32=3, AP_PARAM_FLOAT=4 };
class AP_Param { public: struct ConversionInfo { uint16_t old_key; uint32_t old_group_element;
    ap_var_type type; const char *new_name; }; static bool find_old_parameter(const ConversionInfo *, AP_Param *); };
template<typename T> class Value : public AP_Param { public: T value{}; T get() const { return value; } };
using AP_Int8=Value<int8_t>; using AP_Int16=Value<int16_t>; using AP_Int32=Value<int32_t>; using AP_Float=Value<float>;
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source = tmp_path / 'host.cpp'
    source.write_text(r'''
#include "BindingOldParameter.h"
bool AP_Param::find_old_parameter(const ConversionInfo *info, AP_Param *value) {
    if (info->old_key != 17 || info->old_group_element != 0x12345678) return false;
    switch (info->type) {
    case AP_PARAM_INT8: static_cast<AP_Int8*>(value)->value=-8; break;
    case AP_PARAM_INT16: static_cast<AP_Int16*>(value)->value=-1600; break;
    case AP_PARAM_INT32: static_cast<AP_Int32*>(value)->value=320000; break;
    case AP_PARAM_FLOAT: static_cast<AP_Float*>(value)->value=4.5f; break;
    default: return false;
    } return true;
}
extern "C" int call(const uint8_t *input,uint8_t *output) {
    return AP_Module::binding_parameter_old_value(input,7,output,5);
}
''')
    library = tmp_path / 'old.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-shared', '-fPIC', '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
                    str(source), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    for parameter_type, expected in ((1, -8.0), (2, -1600.0), (3, 320000.0), (4, 4.5)):
        packet = struct.pack('<HIB', 17, 0x12345678, parameter_type)
        output = ctypes.create_string_buffer(5)
        assert lib.call(packet, output) == 0 and output.raw[0] == 1
        assert struct.unpack('<f', output.raw[1:])[0] == expected
    missing = struct.pack('<HIB', 18, 0x12345678, 4)
    output = ctypes.create_string_buffer(5)
    assert lib.call(missing, output) == 0 and output.raw == bytes(5)

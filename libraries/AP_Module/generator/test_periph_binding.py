# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise AP_Periph's owned 64-bit notification-state adapter."""

import ctypes
import os
import subprocess

import bindings
import periph_binding


def test_selection_and_rust_contract(tmp_path):
    spec = periph_binding.select(bindings.read_ir(), 900)
    assert spec['methods'][0]['result'] == 'u64-le'
    generated = periph_binding.rust(spec, '')
    assert "impl crate::bindings::Periph<'_, '_>" in generated
    assert 'pub fn get_vehicle_state(&self)' in generated
    source = tmp_path / 'periph.rs'
    source.write_text('''#![no_std]
pub enum Error { Host } pub type Result<T=()> = core::result::Result<T,Error>;
pub fn result(v:i32)->Result {if v==0 {Ok(())} else {Err(Error::Host)}}
pub struct Api { pub binding_call:Option<unsafe extern "C" fn(*mut core::ffi::c_void,u32,*const u8,u32,*mut u8,u32)->i32> }
pub struct Context<'a>{raw:*mut core::ffi::c_void,api:&'a Api}
pub mod bindings {
    use super::Context;
    pub struct Periph<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
pub mod imports {pub struct Import; impl Import {pub const fn slot(&self)->u32{0}}
pub static BINDING_900:Import=Import;}
mod generated {
''' + generated + '}\n')
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--crate-type', 'lib', '--edition=2021',
                    str(source), '-o', str(tmp_path / 'periph.rlib')], check=True)


def test_host_u64_wire_and_disabled_guard(tmp_path):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '#pragma once\n#define AP_MODULE_ENABLED 1\n',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &)=delete; C &operator=(const C &)=delete
namespace AP_HAL { class Semaphore { public: bool take(uint32_t){return true;} void give(){} }; }
''',
        'Tools/AP_Periph/AP_Periph.h': '''#pragma once
#include <stdint.h>
class AP_Periph_FW { public: static AP_Periph_FW *get_singleton(); uint64_t get_vehicle_state(); };
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    (tmp_path / 'libraries').mkdir()
    source = tmp_path / 'host.cpp'
    source.write_text(r'''
#include "BindingPeriph.h"
static AP_Periph_FW instance;
AP_Periph_FW *AP_Periph_FW::get_singleton() { return &instance; }
uint64_t AP_Periph_FW::get_vehicle_state() { return UINT64_C(0xFEDCBA9876543210); }
extern "C" int call(uint8_t *output) {
    return AP_Module::binding_periph_get_vehicle_state(nullptr,0,output,8);
}
''')
    library = tmp_path / 'periph.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-shared', '-fPIC', '-DHAL_BUILD_AP_PERIPH=1', '-DAP_PERIPH_NOTIFY_ENABLED=1',
                    '-I' + str(tmp_path / 'libraries'), '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
                    str(source), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_void_p]
    output = ctypes.create_string_buffer(8)
    assert lib.call(output) == 0
    assert output.raw == bytes.fromhex('1032547698badcfe')

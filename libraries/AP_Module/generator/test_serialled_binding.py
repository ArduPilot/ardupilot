# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate persistent serial-LED setup ownership and Rust emission."""

import copy
import ctypes
import os
import subprocess

import bindings
import pytest
import serialled_binding as serialled


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert serialled.select(ir, 600) == serialled.specification(600)
    for name in serialled.METHODS:
        changed = copy.deepcopy(ir)
        owner = next(item for item in changed['singletons'] if item['name'] == serialled.OWNER)
        method = next(item for item in owner['methods'] if item['name'] == name)
        method['arguments'][0]['pointer'] = True
        with pytest.raises(ValueError, match=f'unaudited AP_SerialLED.{name}'):
            serialled.select(changed, 600)


def test_contract_is_bounded_and_restart_aware():
    methods = serialled.specification(600)['methods']
    assert [method['id'] for method in methods] == [600, 601, 602]
    assert [method['name'] for method in methods] == [
        'serialLED.set_num_neopixel',
        'serialLED.set_num_neopixel_rgb',
        'serialLED.set_num_profiled',
    ]
    assert all('module-context-channel-claim-on-success' in method['policy'] for method in methods)
    assert all('persistent-driver-allocation-not-released' in method['policy'] for method in methods)


def test_emitted_rust_sdk_wire_and_validation(tmp_path):
    generated = serialled.rust(serialled.specification(600), '')
    source = tmp_path / 'serialled.rs'
    source.write_text(r'''
use core::ffi::c_void;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error { Invalid, Host }
pub type Result<T = ()> = core::result::Result<T, Error>;
fn result(status: i32) -> Result { if status == 0 { Ok(()) } else { Err(Error::Host) } }
struct HostApi {
    binding_call: Option<unsafe extern "C" fn(*mut c_void, u32, *const u8, u32, *mut u8, u32) -> i32>,
}
pub struct Context<'a> { api: &'a HostApi, raw: *mut c_void }
pub mod bindings {
    pub struct SerialLed<'context, 'api> {
        pub(crate) context: &'context crate::Context<'api>,
    }
}
impl<'api> Context<'api> {
    pub fn serial_led(&self) -> bindings::SerialLed<'_, 'api> {
        bindings::SerialLed { context: self }
    }
}
pub(crate) struct Import(u32);
impl Import { pub(crate) fn slot(&self) -> u32 { self.0 } }
pub(crate) mod imports {
    use super::Import;
    pub(crate) static BINDING_600: Import = Import(600);
    pub(crate) static BINDING_601: Import = Import(601);
    pub(crate) static BINDING_602: Import = Import(602);
}
mod generated {
''' + generated + r'''
}

#[cfg(test)]
mod tests {
    use super::{generated::*, Context, Error, HostApi};
    use core::ffi::c_void;

    struct Host { id: u32, input: [u8; 2], calls: u32, output: u8 }
    unsafe extern "C" fn call(raw: *mut c_void, id: u32, input: *const u8, ni: u32,
                               output: *mut u8, no: u32) -> i32 {
        let host = &mut *raw.cast::<Host>();
        host.id = id; host.calls += 1;
        host.input.copy_from_slice(core::slice::from_raw_parts(input, ni as usize));
        core::slice::from_raw_parts_mut(output, no as usize)[0] = host.output;
        0
    }

    #[test]
    fn exact_wire_and_validation() {
        let mut host = Host { id: 0, input: [0; 2], calls: 0, output: 1 };
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        assert_eq!(context.serial_led().set_num_profiled(3, 8), Ok(true));
        assert_eq!((host.id, host.input), (602, [3, 8]));
        assert_eq!(context.serial_led().set_num_neopixel(4, 128), Ok(true));
        assert_eq!(host.id, 600);
        assert_eq!(context.serial_led().set_num_neopixel_rgb(5, 7), Ok(true));
        assert_eq!(host.id, 601);
        let calls = host.calls;
        assert_eq!(context.serial_led().set_num_profiled(0, 8), Err(Error::Invalid));
        assert_eq!(host.calls, calls);
        host.output = 2;
        assert_eq!(context.serial_led().set_num_profiled(3, 8), Err(Error::Host));
    }
}
''')
    subprocess.run(['rustc', '--edition=2021', '--test', str(source), '-o', str(tmp_path / 'test')], check=True)
    subprocess.run([str(tmp_path / 'test')], check=True)


def _write_common_headers(tmp_path, enabled):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '#pragma once\n#define AP_MODULE_ENABLED 1\n',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { class Semaphore { public: bool take(uint32_t) { return true; } void give() {} }; }
using HAL_Semaphore = AP_HAL::Semaphore;
#define WITH_SEMAPHORE(sem) (void)(sem)
''',
        'AP_SerialLED/AP_SerialLED.h': f'''#pragma once
#define AP_SERIALLED_ENABLED {1 if enabled else 0}
#if AP_SERIALLED_ENABLED
#define AP_SERIALLED_MAX_LEDS 128
#include <stdint.h>
class AP_SerialLED {{
public:
    static AP_SerialLED *get_singleton();
    bool set_num_neopixel(uint8_t channel, uint8_t count);
    bool set_num_neopixel_rgb(uint8_t channel, uint8_t count);
    bool set_num_profiled(uint8_t channel, uint8_t count);
}};
#endif
''',
        'BindingConfig.h': '#pragma once\n',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)


@pytest.fixture
def serialled_host(tmp_path):
    _write_common_headers(tmp_path, True)
    source = tmp_path / 'wrapper.cpp'
    source.write_text(r'''
#include <AP_SerialLED/AP_SerialLED.h>
#include "BindingSerialLED.h"
static AP_SerialLED serialled;
static uint8_t last_method;
AP_SerialLED *AP_SerialLED::get_singleton() { return &serialled; }
bool AP_SerialLED::set_num_neopixel(uint8_t, uint8_t count) { last_method = 1; return count != 99; }
bool AP_SerialLED::set_num_neopixel_rgb(uint8_t, uint8_t count) { last_method = 2; return count != 99; }
bool AP_SerialLED::set_num_profiled(uint8_t, uint8_t count) { last_method = 3; return count != 99; }
extern "C" int32_t call_setup(uint8_t method, void *context, const uint8_t *input, uint32_t size,
                               uint8_t *output, uint32_t output_size) {
    if (method == 1) return AP_Module::binding_serialled_set_num_neopixel(context, input, size, output, output_size);
    if (method == 2) return AP_Module::binding_serialled_set_num_neopixel_rgb(context, input, size, output, output_size);
    return AP_Module::binding_serialled_set_num_profiled(context, input, size, output, output_size);
}
extern "C" void cleanup(void *context) { AP_Module::binding_serialled_cleanup(context); }
extern "C" uint8_t method_seen() { return last_method; }
''')
    library = tmp_path / 'serialled.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source),
                    str(bindings.ROOT / 'BindingSerialLED.cpp'), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call_setup.argtypes = [ctypes.c_uint8, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32,
                               ctypes.c_void_p, ctypes.c_uint32]
    lib.cleanup.argtypes = [ctypes.c_void_p]
    return lib


def test_host_claims_successful_channels_and_releases_only_ownership(serialled_host):
    lib = serialled_host
    owner_a = ctypes.c_uint8()
    owner_b = ctypes.c_uint8()
    a = ctypes.addressof(owner_a)
    b = ctypes.addressof(owner_b)
    output = ctypes.create_string_buffer(1)
    assert lib.call_setup(3, a, b'\1\10', 2, output, 1) == 0 and output.raw == b'\1'
    assert lib.method_seen() == 3
    assert lib.call_setup(1, b, b'\1\10', 2, output, 1) == -5
    assert lib.call_setup(1, a, b'\2\143', 2, output, 1) == 0 and output.raw == b'\0'
    assert lib.call_setup(2, b, b'\2\10', 2, output, 1) == 0 and output.raw == b'\1'
    assert lib.method_seen() == 2
    lib.cleanup(a)
    assert lib.call_setup(1, b, b'\1\10', 2, output, 1) == 0 and output.raw == b'\1'
    for packet in (b'\0\10', b'\21\10', b'\1\201'):
        assert lib.call_setup(3, a, packet, 2, output, 1) == -1


def test_host_compiles_with_serialled_disabled(tmp_path):
    _write_common_headers(tmp_path, False)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsyntax-only',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
                    str(bindings.ROOT / 'BindingSerialLED.cpp')], check=True)

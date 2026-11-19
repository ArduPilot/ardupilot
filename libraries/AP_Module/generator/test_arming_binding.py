# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate module-owned auxiliary arming bindings and lifecycle cleanup."""

import copy
import ctypes
import os
import subprocess

import arming_binding as arming
import bindings
import pytest


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert arming.select(ir, 500) == arming.specification(500)
    for name in ('get_aux_auth_id', 'set_aux_auth_passed', 'set_aux_auth_failed'):
        changed = copy.deepcopy(ir)
        owner = next(item for item in changed['singletons'] if item['name'] == arming.OWNER)
        method = next(item for item in owner['methods'] if item['name'] == name)
        method['arguments'][0]['pointer'] = True
        with pytest.raises(ValueError, match=f'unaudited AP_Arming.{name}'):
            arming.select(changed, 500)


def test_contract_is_bounded_owned_and_lifecycle_cleaned():
    spec = arming.specification(500)
    assert [method['id'] for method in spec['methods']] == [500, 501, 502]
    assert [method['name'] for method in spec['methods']] == [
        'arming.get_aux_auth_id', 'arming.set_aux_auth_passed', 'arming.set_aux_auth_failed',
    ]
    assert 'module-context-owned-id' in spec['methods'][0]['policy']
    assert 'lua-reset-isolation' in spec['methods'][0]['policy']
    runtime = (bindings.ROOT / 'Runtime.cpp').read_text()
    cleanup = runtime.index('binding_arming_cleanup(&slot);')
    release = runtime.index('Module::release(slot.module);', cleanup)
    assert cleanup < release


def test_emitted_rust_sdk_wire_and_validation(tmp_path):
    generated = arming.rust(arming.specification(500), '')
    source = tmp_path / 'arming.rs'
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
    use super::Context;
    pub struct Arming<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
impl<'api> Context<'api> {
    pub fn arming(&self) -> bindings::Arming<'_, 'api> { bindings::Arming { context: self } }
}
pub(crate) struct Import(u32);
impl Import { pub(crate) fn slot(&self) -> u32 { self.0 } }
pub(crate) mod imports {
    use super::Import;
    pub(crate) static BINDING_500: Import = Import(500);
    pub(crate) static BINDING_501: Import = Import(501);
    pub(crate) static BINDING_502: Import = Import(502);
}
mod generated {
''' + generated + r'''
}

#[cfg(test)]
mod tests {
    use super::{generated::*, Context, Error, HostApi};
    use core::ffi::c_void;

    struct Host { id: u32, input: [u8; 43], input_len: usize, output: [u8; 2] }
    unsafe extern "C" fn call(raw: *mut c_void, id: u32, input: *const u8, ni: u32,
                               output: *mut u8, no: u32) -> i32 {
        let host = &mut *raw.cast::<Host>();
        host.id = id; host.input_len = ni as usize;
        if ni != 0 { host.input[..ni as usize].copy_from_slice(core::slice::from_raw_parts(input, ni as usize)); }
        if no != 0 { core::slice::from_raw_parts_mut(output, no as usize).copy_from_slice(&host.output[..no as usize]); }
        0
    }
    fn host() -> Host { Host { id: 0, input: [0; 43], input_len: 0, output: [0; 2] } }

    #[test]
    fn exact_wire_and_validation() {
        let mut host = host();
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        host.output = [1, 2];
        assert_eq!(context.arming().get_aux_auth_id(), Ok(Some(2)));
        assert_eq!((host.id, host.input_len), (500, 0));
        assert_eq!(context.arming().set_aux_auth_passed(2), Ok(()));
        assert_eq!((host.id, host.input_len, host.input[0]), (501, 1, 2));
        assert_eq!(context.arming().set_aux_auth_failed(2, "blocked"), Ok(()));
        assert_eq!((host.id, host.input_len), (502, 43));
        assert_eq!(&host.input[..9], &[2, 7, b'b', b'l', b'o', b'c', b'k', b'e', b'd']);
        assert!(host.input[9..].iter().all(|byte| *byte == 0));
        assert_eq!(context.arming().set_aux_auth_failed(2, &"x".repeat(42)), Err(Error::Invalid));
        assert_eq!(context.arming().set_aux_auth_failed(2, "bad\0message"), Err(Error::Invalid));
        assert_eq!(host.id, 502);
        host.output = [0, 1];
        assert_eq!(context.arming().get_aux_auth_id(), Err(Error::Host));
    }
}
''')
    subprocess.run(['rustc', '--edition=2021', '--test', str(source), '-o', str(tmp_path / 'test')], check=True)
    subprocess.run([str(tmp_path / 'test')], check=True)


@pytest.fixture
def host_service(tmp_path):
    arming_header = tmp_path / 'AP_Arming/AP_Arming.h'
    arming_header.parent.mkdir(parents=True)
    arming_header.write_text(r'''#pragma once
#include <stdint.h>
class AP_Arming {
public:
    static AP_Arming *get_singleton();
    bool get_aux_auth_id(uint8_t &id, const void *owner) {
        for (id = 0; id < 3; id++) {
            if (!active[id]) { active[id] = true; owners[id] = owner; return true; }
        }
        return false;
    }
    bool set_aux_auth_passed(uint8_t id, const void *owner) { return valid(id, owner); }
    bool set_aux_auth_failed(uint8_t id, const char *, const void *owner) { return valid(id, owner); }
    void release_aux_auths(const void *owner) {
        for (uint8_t i = 0; i < 3; i++) {
            if (active[i] && owners[i] == owner) { active[i] = false; owners[i] = nullptr; }
        }
    }
private:
    bool valid(uint8_t id, const void *owner) { return id < 3 && active[id] && owners[id] == owner; }
    bool active[3] {};
    const void *owners[3] {};
};
''')
    config = tmp_path / 'AP_Arming/AP_Arming_config.h'
    config.write_text('#pragma once\n#define AP_ARMING_ENABLED 1\n#define AP_ARMING_AUX_AUTH_ENABLED 1\n')
    scripting = tmp_path / 'AP_Scripting/AP_Scripting_config.h'
    scripting.parent.mkdir(parents=True)
    scripting.write_text('#pragma once\n#define AP_MODULE_ENABLED 1\n')
    boards = tmp_path / 'AP_HAL/AP_HAL_Boards.h'
    boards.parent.mkdir(parents=True)
    boards.write_text('#pragma once\n#define AP_MODULE_ENABLED 1\n')
    semaphores = tmp_path / 'AP_HAL/Semaphores.h'
    semaphores.write_text('''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL {
class Semaphore {
public:
    bool take(uint32_t) { return true; }
    void give() {}
};
}
''')
    source = tmp_path / 'wrapper.cpp'
    source.write_text(r'''#include <AP_Arming/AP_Arming.h>
#include "BindingArming.h"
static AP_Arming arming;
AP_Arming *AP_Arming::get_singleton() { return &arming; }
extern "C" int32_t get(void *context, uint8_t *output) {
    return AP_Module::binding_arming_get_aux_auth_id(context, nullptr, 0, output, 2);
}
extern "C" int32_t passed(void *context, const uint8_t *input, uint32_t size) {
    return AP_Module::binding_arming_set_aux_auth_passed(context, input, size, nullptr, 0);
}
extern "C" int32_t failed(void *context, const uint8_t *input, uint32_t size) {
    return AP_Module::binding_arming_set_aux_auth_failed(context, input, size, nullptr, 0);
}
extern "C" void cleanup(void *context) { AP_Module::binding_arming_cleanup(context); }
''')
    library = tmp_path / 'arming.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source),
                    str(bindings.ROOT / 'BindingArming.cpp'), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.get.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.passed.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32]
    lib.failed.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32]
    lib.cleanup.argtypes = [ctypes.c_void_p]
    return lib


def test_host_service_owns_ids_and_rejects_malformed_wire(host_service):
    lib = host_service
    owner_a = ctypes.c_uint8()
    owner_b = ctypes.c_uint8()
    a = ctypes.addressof(owner_a)
    b = ctypes.addressof(owner_b)
    output = ctypes.create_string_buffer(2)
    assert lib.get(a, output) == 0 and output.raw == b'\1\0'
    assert lib.get(b, output) == 0 and output.raw == b'\1\1'
    assert lib.passed(b, b'\0', 1) == -1
    assert lib.passed(a, b'\0', 1) == 0
    packet = bytes([1, 3]) + b'bad' + bytes(38)
    assert lib.failed(b, packet, len(packet)) == 0
    for invalid in (packet[:-1], bytes([1, 42]) + bytes(41), bytes([1, 3]) + b'a\0c' + bytes(38),
                    bytes([1, 3]) + b'bad' + b'x' + bytes(37)):
        assert lib.failed(b, invalid, len(invalid)) == -1
    lib.cleanup(a)
    assert lib.get(b, output) == 0 and output.raw == b'\1\0'

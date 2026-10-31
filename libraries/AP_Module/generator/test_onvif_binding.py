# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate persistent ONVIF client ownership and Rust emission."""

import copy
import ctypes
import json
import os
import subprocess

import bindings
import call_model
import cpp_bindings
import generate
import onvif_binding as onvif
import pytest


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert onvif.select(ir, 700) == onvif.specification(700)
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['singletons'] if item['name'] == onvif.OWNER)
    method = next(item for item in owner['methods'] if item['name'] == onvif.METHOD)
    method['arguments'][0]['pointer'] = True
    with pytest.raises(ValueError, match='unaudited AP_ONVIF.start'):
        onvif.select(changed, 700)


def test_contract_records_persistent_restart_semantics():
    method = onvif.specification(700)
    assert (method['id'], method['name'], method['result']) == (700, 'onvif.start', 'bool-u8')
    assert 'single-module-owner-claimed-before-start' in method['policy']
    assert 'persistent-client-allocation-not-released' in method['policy']
    assert 'blocking-firmware-network-call' in method['policy']


def test_generated_movement_passes_owner_context(tmp_path):
    _write_common_headers(tmp_path, True)
    contract = bindings.specification()
    generated = bindings.emit(contract)['Bindings_generated.cpp']
    marker = '    case BindingId::ONVIF_SET_ABSOLUTEMOVE:\n'
    dispatch = generated.split(marker, 1)[1].splitlines()[0]
    start = generated.index('// AP_ONVIF::set_absolutemove\n')
    end = generated.index('\n}\n', start) + 3
    source = tmp_path / 'movement.cpp'
    source.write_text('#include <AP_ONVIF/AP_ONVIF.h>\n#include <AP_Module/HostApi.h>\n'
                      '#include "BindingONVIF.h"\n#include "BindingSupport.h"\nnamespace AP_Module {\n' +
                      generated[start:end] + '\n'
                      'int invoke(void *context, const uint8_t *input, uint32_t input_size,\n'
                      '           uint8_t *output, uint32_t output_size) {\n' + dispatch + '\n}\n}\n')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), '-c', str(source),
                    '-o', str(tmp_path / 'movement.o')], check=True)


def test_clients_share_the_string_model():
    spec = bindings.specification()
    call = onvif.client_model(spec['onvif']).calls[0]
    assert call_model.calls(spec)['onvif.start'] == call
    assert [(field.name, field.type, offset) for field, offset in call_model.layout(call.inputs)] == [
        (name, call_model.String(256), index * 256) for index, name in enumerate(('username', 'password', 'hostname'))
    ]
    assert (call.input_size, call.output_size) == (768, 1)
    plan = next(plan for plan in cpp_bindings.describe(spec)[2] if plan[0] == 'onvif.start')
    assert plan[-1] == call
    assert [name for name, _, _ in plan[1]] == ['username', 'password', 'hostname']


def test_emitted_rust_sdk_wire_and_validation(tmp_path):
    generated = onvif.rust(onvif.specification(700), '')
    assert "impl crate::bindings::Onvif<'_, '_>" in generated
    assert '    ///\n' in generated
    assert all(line == line.rstrip() for line in generated.splitlines())
    source = tmp_path / 'onvif.rs'
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
    pub struct Onvif<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
impl<'api> Context<'api> {
    pub fn onvif(&self) -> bindings::Onvif<'_, 'api> { bindings::Onvif { context: self } }
}
pub(crate) struct Import(u32);
impl Import { pub(crate) fn slot(&self) -> u32 { self.0 } }
pub(crate) mod imports { use super::Import; pub(crate) static BINDING_700: Import = Import(700); }
mod generated {
''' + generated + r'''
}

#[cfg(test)]
mod tests {
    use super::{generated::*, Context, Error, HostApi};
    use core::ffi::c_void;

    struct Host { id: u32, calls: u32, input: [u8; 768], output: u8, status: i32 }
    unsafe extern "C" fn call(raw: *mut c_void, id: u32, input: *const u8, ni: u32,
                               output: *mut u8, no: u32) -> i32 {
        let host = &mut *raw.cast::<Host>();
        assert_eq!((ni, no), (768, 1));
        host.id = id; host.calls += 1;
        host.input.copy_from_slice(core::slice::from_raw_parts(input, ni as usize));
        core::slice::from_raw_parts_mut(output, no as usize)[0] = host.output;
        host.status
    }

    fn packet_matches(host: &Host, fields: [&str; 3]) {
        for (index, value) in fields.iter().enumerate() {
            let field = &host.input[index * 256..(index + 1) * 256];
            assert_eq!(&field[..value.len()], value.as_bytes());
            assert!(field[value.len()..].iter().all(|byte| *byte == 0));
        }
    }

    #[test]
    fn exact_wire_and_validation() {
        let mut host = Host { id: 0, calls: 0, input: [0; 768], output: 1, status: 0 };
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        assert_eq!(context.onvif().start("user", "pass", "http://camera"), Ok(true));
        assert_eq!(host.id, 700);
        packet_matches(&host, ["user", "pass", "http://camera"]);
        assert_eq!(ONVIF_STRING_MAX, 255);
        let maximum = "x".repeat(255);
        let utf8_maximum = "é".repeat(127) + "x";
        for index in 0..3 {
            for value in ["", maximum.as_str(), utf8_maximum.as_str()] {
                let mut fields = ["user", "pass", "http://camera"];
                fields[index] = value;
                assert_eq!(context.onvif().start(fields[0], fields[1], fields[2]), Ok(true));
                packet_matches(&host, fields);
            }
        }
        let calls = host.calls;
        let too_long = "x".repeat(256);
        let utf8_too_long = "é".repeat(128);
        for index in 0..3 {
            for value in ["bad\0user", "\0", too_long.as_str(), utf8_too_long.as_str()] {
                let mut fields = ["user", "pass", "host"];
                fields[index] = value;
                assert_eq!(context.onvif().start(fields[0], fields[1], fields[2]), Err(Error::Invalid));
            }
        }
        assert_eq!(host.calls, calls);
        host.output = 0;
        assert_eq!(context.onvif().start("user", "pass", "host"), Ok(false));
        for value in [2, 255] {
            host.output = value;
            assert_eq!(context.onvif().start("user", "pass", "host"), Err(Error::Host));
        }
        host.output = 1; host.status = -3;
        assert_eq!(context.onvif().start("user", "pass", "host"), Err(Error::Host));
    }
}
''')
    subprocess.run(['rustc', '--edition=2021', '--test', str(source), '-o', str(tmp_path / 'test')], check=True)
    subprocess.run([str(tmp_path / 'test')], check=True)


def test_emitted_cpp_sdk_wire_and_validation(tmp_path):
    generated = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), generated)
    source = tmp_path / 'onvif.cpp'
    source.write_text(r'''
#include <ap_module.hpp>
#include <assert.h>
#include <string.h>
using namespace ap_module;
static unsigned calls;
static uint8_t seen[768], reply = 1;
static int32_t status;
static int32_t call(void *, uint32_t, const uint8_t *input, uint32_t ni, uint8_t *output, uint32_t no) {
    assert(ni == 768 && no == 1);
    calls++;
    memcpy(seen, input, sizeof(seen));
    output[0] = reply;
    return status;
}
static void packet_matches(const StringView (&fields)[3]) {
    for (unsigned i = 0; i < 3; i++) {
        const uint8_t *field = seen + i * 256;
        if (fields[i].length) { assert(memcmp(field, fields[i].data, fields[i].length) == 0); }
        for (size_t j = fields[i].length; j < 256; j++) { assert(field[j] == 0); }
    }
}
int main() {
    AP_ModuleHostApi api{}; api.binding_call = call;
    ContextBase base(api, nullptr); Context context(base);
    char maximum[256]; memset(maximum, 'x', sizeof(maximum));
    char utf8[256];
    for (unsigned i = 0; i < 128; i++) { utf8[2 * i] = char(0xc3); utf8[2 * i + 1] = char(0xa9); }
    char utf8_maximum[255]; memcpy(utf8_maximum, utf8, 254); utf8_maximum[254] = 'x';
    StringView valid[] = {StringView(nullptr, 0), StringView(maximum, 255), StringView(utf8_maximum, 255)};
    StringView fields[] = {StringView("user"), StringView("pass"), StringView("http://camera")};
    auto result = context.onvif().start(fields[0], fields[1], fields[2]);
    assert(result && result.value); packet_matches(fields);
    for (unsigned index = 0; index < 3; index++) {
        StringView saved = fields[index];
        for (StringView value : valid) {
            fields[index] = value;
            result = context.onvif().start(fields[0], fields[1], fields[2]);
            assert(result && result.value); packet_matches(fields);
        }
        fields[index] = saved;
    }
    const unsigned before = calls;
    StringView invalid[] = {StringView("bad\0user", 8), StringView("\0", 1), StringView(maximum, 256),
                            StringView(utf8, 256), StringView(nullptr, 1)};
    for (unsigned index = 0; index < 3; index++) {
        StringView saved = fields[index];
        for (StringView value : invalid) {
            fields[index] = value;
            assert(context.onvif().start(fields[0], fields[1], fields[2]).error == Error::Invalid);
        }
        fields[index] = saved;
    }
    assert(calls == before);
    reply = 0;
    result = context.onvif().start(fields[0], fields[1], fields[2]);
    assert(result && !result.value);
    const uint8_t malformed[] = {2, 255};
    for (uint8_t value : malformed) {
        reply = value;
        assert(context.onvif().start(fields[0], fields[1], fields[2]).error == Error::Host);
    }
    reply = 1; status = AP_MODULE_STATUS_STOPPED;
    assert(context.onvif().start(fields[0], fields[1], fields[2]).error == Error::Stopped);
}
''')
    executable = tmp_path / 'onvif'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-Wno-missing-field-initializers', '-fsanitize=address,undefined',
                    '-I' + str(generate.ROOT / 'sdk/cpp'), '-I' + str(generated),
                    '-I' + str(generated / 'sdk/cpp'), str(source), str(generated / 'sdk/cpp/imports.cpp'),
                    '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def _write_common_headers(tmp_path, enabled):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': f'''#pragma once
#define AP_MODULE_ENABLED 1
#define ENABLE_ONVIF {1 if enabled else 0}
''',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { class Semaphore { public: bool take(uint32_t) { return true; } void give() {} }; }
using HAL_Semaphore = AP_HAL::Semaphore;
#define WITH_SEMAPHORE(sem) (void)(sem)
''',
        'AP_ONVIF/AP_ONVIF.h': '''#pragma once
#include <AP_HAL/AP_HAL_Boards.h>
#if ENABLE_ONVIF == 1
class AP_ONVIF {
public:
    static AP_ONVIF *get_singleton();
    bool start(const char *username, const char *password, const char *hostname);
    bool set_absolutemove(float pan, float tilt, float zoom);
};
#endif
''',
        'BindingConfig.h': '#pragma once\n',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)


@pytest.fixture
def onvif_host(tmp_path):
    _write_common_headers(tmp_path, True)
    source = tmp_path / 'wrapper.cpp'
    source.write_text(r'''
#include <AP_ONVIF/AP_ONVIF.h>
#include "BindingONVIF.h"
#include <string.h>
static AP_ONVIF onvif;
static bool start_result;
static uint32_t starts;
static char seen_hostname[256];
AP_ONVIF *AP_ONVIF::get_singleton() { return &onvif; }
bool AP_ONVIF::start(const char *, const char *, const char *hostname) {
    starts++;
    strncpy(seen_hostname, hostname, sizeof(seen_hostname) - 1U);
    return start_result;
}
extern "C" int32_t start(void *context, const uint8_t *input, uint32_t size,
                          uint8_t *output, uint32_t output_size) {
    return AP_Module::binding_onvif_start(context, input, size, output, output_size);
}
extern "C" void cleanup(void *context) { AP_Module::binding_onvif_cleanup(context); }
extern "C" void set_result(bool value) { start_result = value; }
extern "C" uint32_t start_count() { return starts; }
extern "C" const char *hostname_seen() { return seen_hostname; }
''')
    library = tmp_path / 'onvif.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source),
                    str(bindings.ROOT / 'BindingONVIF.cpp'), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.start.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.cleanup.argtypes = [ctypes.c_void_p]
    lib.set_result.argtypes = [ctypes.c_bool]
    lib.hostname_seen.restype = ctypes.c_char_p
    return lib


def _packet(username=b'user', password=b'pass', hostname=b'http://camera'):
    fields = []
    for value in (username, password, hostname):
        assert len(value) <= 255 and b'\0' not in value
        fields.append(value + bytes(256 - len(value)))
    return b''.join(fields)


def test_host_claims_before_failed_start_and_releases_only_ownership(onvif_host):
    lib = onvif_host
    owner_a = ctypes.c_uint8()
    owner_b = ctypes.c_uint8()
    a = ctypes.addressof(owner_a)
    b = ctypes.addressof(owner_b)
    output = ctypes.create_string_buffer(1)
    packet = _packet()
    lib.set_result(False)
    assert lib.start(a, packet, len(packet), output, 1) == 0 and output.raw == b'\0'
    assert lib.start_count() == 1
    assert lib.start(b, packet, len(packet), output, 1) == -5
    assert lib.start_count() == 1
    lib.cleanup(a)
    lib.set_result(True)
    packet = _packet(hostname=b'http://new-camera')
    assert lib.start(b, packet, len(packet), output, 1) == 0 and output.raw == b'\1'
    assert lib.hostname_seen() == b'http://new-camera'
    malformed = bytearray(packet)
    malformed[10] = 1
    assert lib.start(a, bytes(malformed), len(malformed), output, 1) == -1


def test_host_compiles_with_onvif_disabled(tmp_path):
    _write_common_headers(tmp_path, False)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsyntax-only',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
                    str(bindings.ROOT / 'BindingONVIF.cpp')], check=True)

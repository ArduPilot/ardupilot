# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate the manual COMMAND_INT contract, Rust wire API and host adapter."""

import copy
import ctypes
import os
import struct
import subprocess

import bindings
import command_int_binding as command_int
import pytest


def packet(command=0, p=(0.0, 0.0, 0.0, 0.0), x=0, y=0, z=0.0, frame=0):
    return struct.pack('<HffffiifB', command, *p, x, y, z, frame)


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert command_int.select(ir, 417) == command_int.specification(417)
    owner = next(item for item in ir['singletons'] if item['name'] == 'GCS')
    alias = next(item for item in owner['aliases'] if item['alias'] == 'run_command_int')
    for field, value in [('manual', False), ('name', 'changed'), ('depends', 'FEATURE')]:
        changed = copy.deepcopy(ir)
        changed_owner = next(item for item in changed['singletons'] if item['name'] == 'GCS')
        changed_alias = next(item for item in changed_owner['aliases'] if item['alias'] == 'run_command_int')
        changed_alias[field] = value
        with pytest.raises(ValueError, match='unaudited GCS.run_command_int'):
            command_int.select(changed, 417)
    assert alias == dict(name='lua_GCS_command_int', alias='run_command_int', depends=None, manual=True)


@pytest.fixture(params=[True, False])
def host(tmp_path, request):
    semaphore = tmp_path / 'AP_HAL/Semaphores.h'
    semaphore.parent.mkdir()
    semaphore.write_text('''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore {
    bool available = true; unsigned depth = 0;
    bool take(uint32_t timeout) {
        if (timeout != 0) { __builtin_trap(); }
        depth++; return true;
    }
    void give() { if (depth != 1) { __builtin_trap(); } depth--; }
}; }
''')
    method = command_int.specification(77)
    source = tmp_path / 'command.cpp'
    library = tmp_path / 'command.so'
    lines = ['#include "BindingSupport.h"', '#include "HostApi.h"', '''
enum MAV_RESULT : uint8_t { MAV_RESULT_ACCEPTED=0, MAV_RESULT_FAILED=4, MAV_RESULT_WIDE=255 };
struct mavlink_command_int_t {
    float param1, param2, param3, param4; int32_t x, y; float z;
    uint16_t command; uint8_t target_system, target_component, frame, current, autocontinue;
};
static AP_HAL::Semaphore sem;
static bool gcs_present = true, scheduler_present = true;
static unsigned calls;
static MAV_RESULT configured_result = MAV_RESULT_ACCEPTED;
static mavlink_command_int_t received;
struct GCS {
    static GCS *get_singleton() { static GCS gcs; return gcs_present ? &gcs : nullptr; }
    MAV_RESULT lua_command_int_packet(const mavlink_command_int_t &packet) {
        if (sem.depth != 1) { __builtin_trap(); }
        calls++; received=packet; return configured_result;
    }
};
struct AP_Scheduler {
    static AP_Scheduler *get_singleton() { static AP_Scheduler scheduler; return scheduler_present ? &scheduler : nullptr; }
    AP_HAL::Semaphore &get_semaphore() { return sem; }
};
namespace AP_Module {
''']
    lines += command_int.cpp(method)
    lines += ['}', '''
extern "C" int call(const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    return AP_Module::binding_77(in,ni,out,no);
}
extern "C" void configure(bool gcs, bool scheduler, bool available, unsigned result) {
    gcs_present=gcs; scheduler_present=scheduler; sem.available=available;
    configured_result=static_cast<MAV_RESULT>(result);
}
extern "C" unsigned count() { return calls; }
extern "C" unsigned held() { return sem.depth; }
extern "C" void read_packet(uint8_t *out) {
    AP_Module::BindingWire::write(out,received.command,2);
    AP_Module::BindingWire::write_float(out+2,received.param1);
    AP_Module::BindingWire::write_float(out+6,received.param2);
    AP_Module::BindingWire::write_float(out+10,received.param3);
    AP_Module::BindingWire::write_float(out+14,received.param4);
    uint32_t x,y; memcpy(&x,&received.x,4); memcpy(&y,&received.y,4);
    AP_Module::BindingWire::write(out+18,x,4); AP_Module::BindingWire::write(out+22,y,4);
    AP_Module::BindingWire::write_float(out+26,received.z); out[30]=received.frame;
}
''']
    source.write_text('\n'.join(lines))
    subprocess.run([
        os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
        f'-DAP_SCRIPTING_ENABLED={int(request.param)}', f'-DHAL_GCS_ENABLED={int(request.param)}',
        '-DAP_SCHEDULER_ENABLED=1', '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
        str(source), '-o', str(library),
    ], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.configure.argtypes = [ctypes.c_bool, ctypes.c_bool, ctypes.c_bool, ctypes.c_uint]
    lib.read_packet.argtypes = [ctypes.c_void_p]
    return lib, request.param


def test_host_exact_copy_defaults_and_float_bits(host):
    lib, enabled = host
    out = ctypes.create_string_buffer(4)
    cases = [
        (packet(), 0),
        (packet(65535, (-0.0, float('inf'), float('-inf'), struct.unpack('<f', b'!#\xc1\x7f')[0]),
                -0x80000000, 0x7fffffff, -0.0, 255), 255),
    ]
    for data, result_value in cases:
        lib.configure(True, True, True, result_value)
        out.raw = b'xxxx'
        assert lib.call(data, len(data), out, 4) == (0 if enabled else -3)
        assert out.raw == (struct.pack('<i', result_value) if enabled else bytes(4))
        if enabled:
            copied = ctypes.create_string_buffer(command_int.WIRE_SIZE)
            lib.read_packet(copied)
            assert copied.raw == data
        assert lib.held() == 0
    assert lib.count() == (2 if enabled else 0)


def test_host_availability_and_lock(host):
    lib, enabled = host
    data = packet(22, (1.0, 2.0, 3.0, 4.0), -3, 5, 6.0, 3)
    guarded = ctypes.create_string_buffer(b'x' * 6, 6)
    out = ctypes.byref(guarded, 1)
    for config, expected in [((False, True, True, 0), -3), ((True, False, True, 0), -3),
                             ((True, True, False, 0), 0)]:
        lib.configure(*config)
        before = lib.count()
        expected_status = expected if enabled else -3
        assert lib.call(data, 31, out, 4) == expected_status
        assert lib.count() == before + (expected_status == 0)
        assert lib.held() == 0
        assert guarded.raw[:1] == guarded.raw[-1:] == b'x'


def test_rust_sdk_shape_and_zero_defaults():
    source = command_int.rust(command_int.specification(77), '')
    assert '#[derive(Clone, Copy, Debug, Default, PartialEq)]' in source
    assert 'pub struct CommandIntParams' in source
    assert "impl crate::bindings::Gcs<'_, '_>" in source
    assert 'pub fn run_command_int' in source
    assert 'pub fn gcs_run_command_int' not in source
    assert 'Result<crate::enums::MavResult>' in source
    assert 'let mut input = [0u8; 31];' in source
    assert 'BINDING_77.slot()' in source
    assert 'MavResult(i32::from_le_bytes(output))' in source


def test_emitted_rust_sdk_wire_and_status(tmp_path):
    generated = command_int.rust(command_int.specification(77), '')
    source = tmp_path / 'command_int.rs'
    source.write_text('''
use core::ffi::c_void;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error { Stopped, Invalid, Unavailable, Busy, Host }
pub type Result<T = ()> = core::result::Result<T, Error>;
fn result(status: i32) -> Result {
    match status {
        0 => Ok(()), 1 => Err(Error::Stopped), -1 => Err(Error::Invalid),
        -3 => Err(Error::Unavailable), -5 => Err(Error::Busy), _ => Err(Error::Host),
    }
}
struct HostApi {
    binding_call: Option<unsafe extern "C" fn(*mut c_void, u32, *const u8, u32, *mut u8, u32) -> i32>,
}
pub struct Context<'a> { api: &'a HostApi, raw: *mut c_void }
pub mod bindings {
    use super::Context;
    pub struct Gcs<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
impl<'api> Context<'api> {
    pub fn gcs(&self) -> bindings::Gcs<'_, 'api> { bindings::Gcs { context: self } }
}
pub(crate) struct Import;
impl Import { pub(crate) fn slot(&self) -> u32 { 77 } }
pub(crate) mod imports { pub(crate) static BINDING_77: super::Import = super::Import; }
pub mod enums {
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct MavResult(pub i32);
}
mod command_int {
''' + generated + '''
}

#[cfg(test)]
mod tests {
    use super::{command_int::CommandIntParams, enums::MavResult, Context, Error, HostApi};
    use core::ffi::c_void;

    struct Host { status: i32, result: i32, seen: [u8; 31], calls: u32 }

    unsafe extern "C" fn call(raw: *mut c_void, id: u32, input: *const u8, ni: u32,
                               output: *mut u8, no: u32) -> i32 {
        let host = &mut *raw.cast::<Host>();
        assert_eq!((id, ni, no), (77, 31, 4));
        host.seen.copy_from_slice(core::slice::from_raw_parts(input, ni as usize));
        core::slice::from_raw_parts_mut(output, no as usize).copy_from_slice(&host.result.to_le_bytes());
        host.calls += 1;
        host.status
    }

    fn invoke(host: &mut Host, command: u16, params: &CommandIntParams) -> super::Result<MavResult> {
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (host as *mut Host).cast() };
        context.gcs().run_command_int(command, params)
    }

    #[test]
    fn zero_defaults_and_exact_fields() {
        let mut host = Host { status: 0, result: 4, seen: [255; 31], calls: 0 };
        assert_eq!(invoke(&mut host, 22, &CommandIntParams::default()), Ok(MavResult(4)));
        let mut expected = [0; 31]; expected[..2].copy_from_slice(&22u16.to_le_bytes());
        assert_eq!(host.seen, expected);

        let params = CommandIntParams {
            p1: -0.0, p2: f32::from_bits(0x7fc12345), p3: f32::INFINITY,
            p4: f32::NEG_INFINITY, x: i32::MIN, y: i32::MAX,
            z: f32::from_bits(0x80000001), frame: u8::MAX,
        };
        host.result = 255;
        assert_eq!(invoke(&mut host, u16::MAX, &params), Ok(MavResult(255)));
        assert_eq!(&host.seen[..2], &u16::MAX.to_le_bytes());
        for (offset, bits) in [(2, 0x80000000u32), (6, 0x7fc12345), (10, 0x7f800000),
                               (14, 0xff800000), (26, 0x80000001)] {
            assert_eq!(&host.seen[offset..offset+4], &bits.to_le_bytes());
        }
        assert_eq!(&host.seen[18..22], &i32::MIN.to_le_bytes());
        assert_eq!(&host.seen[22..26], &i32::MAX.to_le_bytes());
        assert_eq!(host.seen[30], 255);
        assert_eq!(host.calls, 2);
    }

    #[test]
    fn host_status_is_propagated() {
        let mut host = Host { status: 0, result: 0, seen: [0; 31], calls: 0 };
        for (status, error) in [(1, Error::Stopped), (-1, Error::Invalid),
                                (-3, Error::Unavailable), (-5, Error::Busy), (-99, Error::Host)] {
            host.status = status;
            assert_eq!(invoke(&mut host, 0, &CommandIntParams::default()), Err(error));
        }
    }
}
''')
    binary = tmp_path / 'command_int_tests'
    subprocess.run([
        os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--test', str(source), '-o', str(binary),
    ], check=True)
    subprocess.run([str(binary), '--quiet'], check=True)


def test_policy_requires_scheduler_lock_and_owned_copy():
    policy = command_int.specification(1)['policy']
    assert 'owned-command-int-copy' in policy
    assert 'zero-default-params' in policy
    assert 'blocking-scheduler-semaphore' in policy
    source = '\n'.join(command_int.cpp(command_int.specification(1)))
    assert 'BindingLock lock(scheduler->get_semaphore())' in source
    assert source.index('BindingLock lock') < source.index('lua_command_int_packet(packet)')

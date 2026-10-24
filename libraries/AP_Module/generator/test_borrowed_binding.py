# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Indexed-object strategy: never trust module-supplied firmware pointers."""

import ctypes
import os
import struct
import subprocess

import bindings
import borrowed_binding
import pytest
import singleton_binding


@pytest.fixture(params=[True, False])
def handles(tmp_path, request):
    ir = bindings.read_ir()
    objects, _ = borrowed_binding.select(ir, 1, bindings.method_contract, bindings.policies()['effects'])
    rc = next(o for o in ir['singletons'] if o['name'] == 'RC_Channels')
    query = bindings.method_contract(rc, next(m for m in rc['methods'] if m['name'] == 'lua_rc_channel'))
    query['id'] = 10
    methods = objects + [query]
    header = tmp_path / 'AP_HAL/Semaphores.h'
    header.parent.mkdir()
    header.write_text('''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore {
    bool available = true;
    unsigned held = 0;
    bool take(uint32_t timeout) { if (timeout != 0) { __builtin_trap(); } held++; return true; }
    void give() { if (held != 1) { __builtin_trap(); } held--; }
}; }
''')
    source, binary = tmp_path / 'borrowed.cpp', tmp_path / 'borrowed.so'
    lines = ['#include "BindingSupport.h"', '#include "HostApi.h"', '#define NUM_RC_CHANNELS 3', '''
static AP_HAL::Semaphore sem;
static unsigned calls;
static bool present = true, scheduler_present = true;
static int missing = -1;
struct RC_Channel {
    unsigned index;
    void check() { if (sem.held != 1) { __builtin_trap(); } calls++; }
    float norm_input() { check(); return float(index); }
    float norm_input_dz() { check(); return -float(index); }
    float norm_input_ignore_trim() { check(); return float(index) / 2; }
    uint8_t get_aux_switch_pos() { check(); return index; }
    void set_override(uint16_t value, uint32_t timestamp) {
        check(); if (timestamp != 0 || value > 2200) { __builtin_trap(); }
    }
};
struct RC_Channels {
    static RC_Channels *get_singleton() { static RC_Channels rc; return present ? &rc : nullptr; }
    RC_Channel *channel(uint32_t i) {
        static RC_Channel objects[3] = {{0}, {1}, {2}};
        if (sem.held != 1) { __builtin_trap(); }
        return i < 3 && int(i) != missing ? &objects[i] : nullptr;
    }
    RC_Channel *lua_rc_channel(uint8_t i) {
        static RC_Channel rogue{99};
        return missing == -2 ? &rogue : channel(i - 1);
    }
};
struct AP_Scheduler {
    static AP_Scheduler *get_singleton() { static AP_Scheduler s; return scheduler_present ? &s : nullptr; }
    AP_HAL::Semaphore &get_semaphore() { return sem; }
};
namespace AP_Module {
''']
    lines += borrowed_binding.cpp_helpers({'RC_Channel'})
    for method in methods:
        lines += singleton_binding.cpp(method)
    lines += ['}', '''extern "C" int call(unsigned id, const uint8_t *in, unsigned ni, uint8_t *out, unsigned no) {
    switch (id) {''']
    lines += [f'    case {m["id"]}: return AP_Module::binding_{m["id"]}(in, ni, out, no);' for m in methods]
    lines += ['    default: return -1;', '    }', '}', '''
extern "C" void configure(bool owner, bool lock, bool scheduler, int null_index) {
    present=owner; sem.available=lock; scheduler_present=scheduler; missing=null_index;
}
extern "C" unsigned count() { return calls; }
''']
    source.write_text('\n'.join(lines))
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-Wno-unused-variable', f'-DAP_RC_CHANNEL_ENABLED={int(request.param)}', '-DAP_SCHEDULER_ENABLED=1',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source), '-o', str(binary)], check=True)
    lib = ctypes.CDLL(str(binary))
    lib.call.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.call.restype = ctypes.c_int32
    lib.configure.argtypes = [ctypes.c_bool, ctypes.c_bool, ctypes.c_bool, ctypes.c_int]
    return lib, methods, request.param


def test_handle_roundtrip_and_validation(handles):
    lib, methods, enabled = handles
    out = ctypes.create_string_buffer(4)
    for token in (1, 2, 3):
        assert lib.call(10, bytes([token]), 1, out, 4) == (0 if enabled else -3)
        assert out.raw == struct.pack('<I', token if enabled else 0)
    for method in methods[:-1]:
        ni, no = singleton_binding.sizes(method)
        for token in (1, 2, 3):
            packet = struct.pack('<I', token) + (struct.pack('<H', 1600) if ni == 6 else b'')
            assert lib.call(method['id'], packet, ni, out, no) == (0 if enabled else -3)
            if enabled and no == 4:
                expected = {'norm_input': token - 1, 'norm_input_dz': 1 - token,
                            'norm_input_ignore_trim': (token - 1) / 2}[method['method']]
                assert struct.unpack('<f', out.raw)[0] == expected
            for invalid in (0, 4, 0xffffffff):
                before = lib.count()
                assert lib.call(method['id'], struct.pack('<I', invalid) + packet[4:], ni, out, no) == (-1 if enabled else -3)
                assert lib.count() == before
        lib.configure(True, False, True, -1)
        assert lib.call(method['id'], packet, ni, out, no) == (0 if enabled else -3)
        for invalid in (0, 4, 0xffffffff):
            before = lib.count()
            invalid_packet = struct.pack('<I', invalid) + packet[4:]
            assert lib.call(method['id'], invalid_packet, ni, out, no) == (-1 if enabled else -3)
            assert lib.count() == before
        lib.configure(True, True, False, -1)
        assert lib.call(method['id'], packet, ni, out, no) == -3
        lib.configure(False, True, True, -1)
        assert lib.call(method['id'], packet, ni, out, no) == -3
        lib.configure(True, True, True, 2)
        assert lib.call(method['id'], packet, ni, out, no) == -3
        lib.configure(True, True, True, -1)
    lib.configure(True, True, True, 1)
    assert lib.call(10, b'\2', 1, out, 4) == (0 if enabled else -3)
    assert out.raw == bytes(4)
    lib.configure(True, True, True, -2)
    assert lib.call(10, b'\1', 1, out, 4) == (-4 if enabled else -3)
    assert out.raw == bytes(4)


def test_object_category_discovers_new_method():
    ir = bindings.read_ir()
    owner = next(o for o in ir['ap_objects'] if o['name'] == 'RC_Channel')
    owner['methods'].append(dict(owner['methods'][0], name='new_method'))
    methods, reports = borrowed_binding.select(ir, 100, bindings.method_contract, {})
    assert any(m['method'] == 'new_method' for m in methods)
    assert any(r['owner'] == 'SocketAPM' and r['status'] == 'unsupported' for r in reports)

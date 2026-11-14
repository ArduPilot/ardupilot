# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise the real shared-state service with concurrent native callers."""

import ctypes
import json
import os
import struct
import subprocess

from concurrent.futures import ThreadPoolExecutor

import generate
import pytest


@pytest.fixture
def service(tmp_path):
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), tmp_path)
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': r'''#pragma once
#include <stdint.h>
#include <mutex>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL {
struct Semaphore { std::mutex mutex; bool take(uint32_t){mutex.lock();return true;} void give(){mutex.unlock();} };
struct Guard { Semaphore &sem; Guard(Semaphore &s):sem(s){sem.take(0);} ~Guard(){sem.give();} };
}
using HAL_Semaphore=AP_HAL::Semaphore;
#define JOIN2(a,b) a##b
#define JOIN(a,b) JOIN2(a,b)
#define WITH_SEMAPHORE(s) AP_HAL::Guard JOIN(guard_,__LINE__)(s)
''',
    }
    for name, text in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    # Use the generated host headers and the production service, including its registry.
    module = tmp_path / 'AP_Module'
    module.mkdir()
    for name in ('HostApi.h', 'BindingConfig.h'):
        (module / name).write_text((tmp_path / name).read_text())
    source = tmp_path / 'test.cpp'
    source.write_text('#include "' + str(generate.ROOT / 'BindingSharedState.cpp') + '"\n' + r'''
extern "C" int call(unsigned method, void *ctx, const uint8_t *in, unsigned ni, uint8_t *out, unsigned no) {
    using namespace AP_Module;
    switch(method) {
    case 0: return binding_shared_state_open(ctx,in,ni,out,no);
    case 1: return binding_shared_state_read(ctx,in,ni,out,no);
    case 2: return binding_shared_state_write(ctx,in,ni,out,no);
    case 3: return binding_shared_state_close(ctx,in,ni,out,no);
    default: binding_shared_state_cleanup(ctx); return 0;
    }
}
''')
    binary = tmp_path / 'test.so'
    command = [os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
               '-pthread', '-fPIC', '-shared', '-DAP_MODULE_ENABLED=1', '-I' + str(tmp_path),
               str(source), '-o', str(binary)]
    subprocess.run(command, check=True)
    lib = ctypes.CDLL(str(binary))
    lib.call.argtypes = [ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint,
                         ctypes.c_void_p, ctypes.c_uint]
    lib.call.restype = ctypes.c_int

    class Service:
        def call(self, method, context, packet, size):
            out = ctypes.create_string_buffer(size) if size else None
            code = lib.call(method, context, packet, len(packet), out, size)
            return code, out.raw if out is not None else b''

        def open(self, context, block=7, size=2048):
            code, output = self.call(0, context, struct.pack('<II', block, size), 4)
            assert code == 0
            return struct.unpack('<I', output)[0]

        def read(self, context, token, offset=0, size=2048):
            code, output = self.call(1, context, struct.pack('<IIH', token, offset, size), 2056)
            assert code == 0
            assert output[8+size:] == bytes(2048-size)
            return struct.unpack('<Q', output[:8])[0], output[8:8+size]

        def write(self, context, token, data, revision=0, offset=0):
            code, output = self.call(2, context, struct.pack('<IIHQ', token, offset, len(data), revision) +
                                     data.ljust(2048, b'\0'), 8)
            assert code == 0
            return struct.unpack('<Q', output)[0]

        def close(self, context, token):
            assert self.call(3, context, struct.pack('<I', token), 0)[0] == 0

        def cleanup(self, context):
            assert self.call(4, context, b'', 0)[0] == 0

    instance = Service()
    instance.command = command
    instance.binary = binary
    return instance


def test_shared_copy_revision_and_lifetime(service):
    a = service.open(1)
    b = service.open(2)
    assert a != b
    assert service.read(2, b) == (1, bytes(2048))
    assert service.write(1, a, b'hello', revision=1, offset=3) == 2
    assert service.write(2, b, b'stale', revision=1, offset=3) == 0
    assert service.read(2, b, 3, 5) == (2, b'hello')
    service.cleanup(1)
    assert service.read(2, b, 3, 5) == (2, b'hello')
    service.close(2, b)
    c = service.open(1)
    assert c not in (a, b)
    assert service.read(1, c) == (1, bytes(2048))
    assert service.call(1, 1, struct.pack('<IIH', a, 0, 1), 2056)[0] == -1


def test_invalid_handles_sizes_bounds_and_padding(service):
    token = service.open(1)
    assert service.call(0, 2, struct.pack('<II', 7, 1024), 4)[0] == -1
    for size in (0, 16385, 0xffffffff):
        assert service.call(0, 1, struct.pack('<II', 8, size), 4)[0] == -1
    for context, offset, length in ((2, 0, 1), (1, 2048, 1), (1, 0xffffffff, 1), (1, 0, 2049)):
        assert service.call(1, context, struct.pack('<IIH', token, offset, length), 2056)[0] == -1
        packet = struct.pack('<IIHQ', token, offset, length, 0) + bytes(2048)
        assert service.call(2, context, packet, 8)[0] == -1
    packet = struct.pack('<IIHQ', token, 0, 1, 0) + b'a' + bytes(2046) + b'x'
    assert service.call(2, 1, packet, 8)[0] == -1
    assert service.call(3, 2, struct.pack('<I', token), 0)[0] == -1
    assert service.read(1, token) == (1, bytes(2048))
    assert service.write(1, token, b'', offset=2048) == 2


def test_budgets_and_cleanup(service):
    for context in range(1, 5):
        service.open(context, context, 16384)
    assert service.call(0, 5, struct.pack('<II', 5, 1), 4)[0] == -2
    service.cleanup(1)
    service.open(5, 5, 16384)
    for context in range(1, 6):
        service.cleanup(context)
    for context in range(1, 9):
        service.open(context, context, 1)
    assert service.call(0, 9, struct.pack('<II', 9, 1), 4)[0] == -2
    for context in range(1, 9):
        service.cleanup(context)
    for context in range(1, 9):
        for _ in range(4):
            service.open(context)
        assert service.call(0, context, struct.pack('<II', 7, 2048), 4)[0] == -2
    assert service.call(0, 9, struct.pack('<II', 7, 2048), 4)[0] == -2
    service.cleanup(1)
    service.open(9)


def test_mutex_covers_whole_copy_and_cas(service):
    tokens = [service.open(i + 1) for i in range(4)]

    def worker(index):
        context, token = index + 1, tokens[index]
        for _ in range(100):
            while True:
                revision, data = service.read(context, token)
                # Every successful write fills the whole block with the same value.
                assert data == data[:8] * 256
                value = struct.unpack('<Q', data[:8])[0] + 1
                if service.write(context, token, struct.pack('<Q', value) * 256, revision):
                    break

    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(worker, range(4)))
    revision, data = service.read(1, tokens[0])
    assert revision == 401 and data == struct.pack('<Q', 400) * 256


def test_feature_disabled(service):
    binary = service.binary.with_name('disabled.so')
    command = list(service.command)
    command.insert(1, '-DAP_MODULE_SHARED_STATE_ENABLED=0')
    command[-1] = str(binary)
    subprocess.run(command, check=True)
    lib = ctypes.CDLL(str(binary))
    lib.call.argtypes = [ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint,
                         ctypes.c_void_p, ctypes.c_uint]
    for method in range(4):
        assert lib.call(method, None, None, 0, None, 0) == -3
    assert lib.call(4, None, None, 0, None, 0) == 0

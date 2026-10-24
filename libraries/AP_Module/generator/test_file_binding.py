# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Fault-inject the real file adapter, including every descriptor exit path."""

import ctypes
import os
import struct
import subprocess

import bindings
import file_binding
import pytest


def packet(path=b'fixture', offset=0, count=8):
    return path.ljust(256, b'\0') + struct.pack('<IH', offset, count)


@pytest.fixture(params=[True, False])
def filesystem(tmp_path, request):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t) { __builtin_trap(); } void give() { __builtin_trap(); } }; }
''',
        'AP_Filesystem/AP_Filesystem.h': '''#pragma once
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>
struct AP_Filesystem {
    int open(const char *, int); int close(int);
    int32_t lseek(int, int32_t, int); int32_t read(int, void *, uint32_t);
};
namespace AP { AP_Filesystem &FS(); }
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source, library = tmp_path / 'file.cpp', tmp_path / 'file.so'
    source.write_text('''#include "BindingFile.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <string.h>
static unsigned mode, opens, closes, live, reads, offset;
static int open_fd;
AP_Filesystem &AP::FS() { static AP_Filesystem fs; return fs; }
int AP_Filesystem::open(const char *path, int flags) {
    if (flags != O_RDONLY || live != 0) { __builtin_trap(); }
    opens++; reads=0;
    if (!strcmp(path, "missing") || !strcmp(path, "denied")) { return -1; }
    live++; open_fd = mode == 9 ? 0 : 7; return open_fd;
}
int AP_Filesystem::close(int fd) {
    if (fd != open_fd || live != 1) { __builtin_trap(); }
    closes++; live--; return mode == 7 ? -1 : 0;
}
int32_t AP_Filesystem::lseek(int fd, int32_t pos, int whence) {
    if (fd != open_fd || live != 1 || whence != SEEK_SET || pos < 0) { __builtin_trap(); }
    if (mode == 1) { return -1; }
    if (mode == 2) { offset=0; return 0; }
    if (mode == 3) { return pos+1; }
    offset = (mode == 8 && pos > 9) ? 9 : pos; return offset;
}
int32_t AP_Filesystem::read(int fd, void *out, uint32_t count) {
    if (fd != open_fd || live != 1 || count == 0 || count > 256) { __builtin_trap(); }
    reads++;
    if (mode == 4 || (mode == 5 && reads > 1)) { return -1; }
    if (mode == 6) { return count+1; }
    if (offset >= 9) { return 0; }
    unsigned n = count < 3 ? count : 3;
    if (n > 9-offset) { n=9-offset; }
    memcpy(out, &"123456789"[offset], n); offset+=n; return n;
}
extern "C" int call(const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    return AP_Module::binding_file_read_at(in, ni, out, no);
}
extern "C" void configure(unsigned m) { mode=m; }
extern "C" unsigned open_count() { return opens; }
extern "C" unsigned close_count() { return closes; }
extern "C" unsigned live_count() { return live; }
''')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-DAP_MODULE_ENABLED=1', f'-DAP_FILESYSTEM_FILE_READING_ENABLED={int(request.param)}',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(bindings.ROOT / 'BindingFile.cpp'),
                    str(source), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    return lib, request.param


def test_read_chunks_and_eof(filesystem):
    lib, enabled = filesystem
    out = ctypes.create_string_buffer(259)
    for mode in (0, 8, 9):  # POSIX-style, FatFS clamped seeks, and valid fd zero
        lib.configure(mode)
        for offset, count, expected in [(0, 8, b'12345678'), (3, 4, b'4567'), (8, 256, b'9'),
                                        (9, 1, b''), (100, 4, b''), (0x7fffffff - 8, 8, b'')]:
            data = packet(offset=offset, count=count)
            out.raw = b'\xff' * 259
            assert lib.call(data, len(data), out, 259) == (0 if enabled else -3)
            assert out.raw == ((b'\1' + struct.pack('<H', len(expected)) + expected.ljust(256, b'\0'))
                               if enabled else bytes(259))
            assert lib.live_count() == 0
    for path in (b'missing', b'denied'):
        assert lib.call(packet(path), 262, out, 259) == (0 if enabled else -3)
        assert out.raw == bytes(259)
    assert lib.close_count() == (18 if enabled else 0)


def test_failures_release_descriptors(filesystem):
    lib, enabled = filesystem
    out = ctypes.create_string_buffer(259)
    for mode in range(1, 8):
        lib.configure(mode)
        for _ in range(40):  # above FatFS's 16 descriptor slots, on every error path
            out.raw = b'\xff' * 259
            assert lib.call(packet(offset=1), 262, out, 259) == (-4 if enabled else -3)
            assert out.raw == bytes(259)
            assert lib.live_count() == 0
    assert lib.open_count() == lib.close_count() == (280 if enabled else 0)


def test_invalid_packets_do_not_open(filesystem):
    lib, enabled = filesystem
    out = ctypes.create_string_buffer(259)
    bad = [packet(b''), packet(b'@SYS'), packet(b'/@MISSION'), packet(b'//@unknown'),
           packet(b'a\0b'), packet(b'x' * 256), packet(count=0), packet(count=257),
           packet(offset=0x80000000), packet(offset=0x7fffffff), packet(offset=0xffffffff)]
    for data in bad:
        assert lib.call(data, 262, out, 259) == (-1 if enabled else -3)
        assert out.raw == bytes(259)
    for ni, no in ((261, 259), (263, 259), (262, 258), (262, 260)):
        out.raw = b'\xff' * 259
        assert lib.call(packet(), ni, out, no) == -1
        assert out.raw == b'\xff' * 259
    assert lib.call(None, 262, out, 259) == -1
    assert lib.call(packet(), 262, None, 259) == -1
    assert lib.open_count() == 0


def test_file_contract_has_no_scheduler_lock():
    assert 'no-scheduler-lock' in file_binding.specification(1)['policy']
    source = (bindings.ROOT / 'BindingFile.cpp').read_text()
    assert 'BindingLock' not in source and 'AP_Scheduler' not in source

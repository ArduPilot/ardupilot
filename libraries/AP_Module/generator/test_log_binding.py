# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate real logger adapter packets before registration or log writes."""

import ctypes
import os
import struct
import subprocess

import bindings
import pytest


def packet(fmt=b'f', payload=struct.pack('<f', 1.25), name=b'RLOG', labels=b'Value', units=None):
    def text(value, width):
        return value.ljust(width, b'\0')
    u, m = units if units is not None else (b'', b'')
    return (text(name, 5) + text(labels, 58) + text(fmt, 16) + text(u, 16) + text(m, 16) +
            bytes([units is not None]) + struct.pack('<H', len(payload)) + payload.ljust(244, b'\0'))


@pytest.fixture(params=[True, False])
def logger(tmp_path, request):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/AP_HAL.h': '#pragma once\n#include <stdint.h>\nnamespace AP_HAL { uint64_t micros64(); }\n',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t) { __builtin_trap(); } void give() { __builtin_trap(); } }; }
''',
        'AP_Logger/AP_Logger.h': '''#pragma once
#include <stdint.h>
#define LOG_PACKET_MAX_LEN 255
#define HEAD_BYTE1 0xA3
#define HEAD_BYTE2 0x95
struct UnitStructure { char ID; };
struct MultiplierStructure { char ID; };
static const UnitStructure log_Units[] = {{'s'}, {'m'}, {'-'}, {'?'}, {'#'}, {'D'}, {'U'}};
static const MultiplierStructure log_Multipliers[] = {{'F'}, {'G'}, {'-'}, {'?'}, {'0'}};
struct AP_Logger {
    struct log_write_fmt {
        uint8_t msg_type, msg_len;
        const char *name, *fmt, *labels, *units, *mults;
    };
    static AP_Logger *get_singleton();
    int16_t Write_calc_msg_len(const char *);
    log_write_fmt *msg_fmt_for_name(const char *, const char *, const char *, const char *, const char *, bool, bool);
    void Safe_Write_Emit_FMT(log_write_fmt *);
    void WriteBlock(const void *, uint16_t);
};
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source, library = tmp_path / 'log.cpp', tmp_path / 'log.so'
    source.write_text('''#include "BindingLog.h"
#include "BindingLog_generated.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/AP_HAL.h>
#include <string.h>
static unsigned mode, calls, writes, emits, last_length;
static uint8_t record[255];
static char name_copy[5], fmt_copy[17], labels_copy[65], units_copy[17], mults_copy[17];
uint64_t AP_HAL::micros64() { return UINT64_C(0x1122334455667788); }
AP_Logger *AP_Logger::get_singleton() { static AP_Logger logger; return mode == 1 ? nullptr : &logger; }
int16_t AP_Logger::Write_calc_msg_len(const char *fmt) {
    int16_t n=3;
    for (const char *p=fmt; *p; p++) { unsigned size=AP_Module::log_field_size(*p); if (!size) { return -1; } n+=size; }
    return n;
}
AP_Logger::log_write_fmt *AP_Logger::msg_fmt_for_name(const char *name, const char *labels,
    const char *units, const char *mults, const char *fmt, bool direct, bool copy) {
    if (!direct || !copy) { __builtin_trap(); }
    calls++;
    if (mode == 2) { return nullptr; }
    strcpy(name_copy,name); strcpy(fmt_copy,fmt); strcpy(labels_copy,labels);
    strcpy(units_copy,units ? units : ""); strcpy(mults_copy,mults ? mults : "");
    static log_write_fmt result;
    result={42, uint8_t(Write_calc_msg_len(fmt)), name_copy, fmt_copy, labels_copy,
            units ? units_copy : nullptr, mults ? mults_copy : nullptr};
    if (mode == 3) { result.fmt="QB"; }
    if (mode == 4) { result.labels="TimeUS,Other"; }
    if (mode == 5) { result.units=nullptr; }
    if (mode == 6) { result.msg_len++; }
    if (mode == 7) { result.name="OOPS"; }
    if (mode == 8) { result.mults=nullptr; }
    if (mode == 9) { result.units="s-"; }
    if (mode == 10) { result.mults="F-"; }
    return &result;
}
void AP_Logger::Safe_Write_Emit_FMT(log_write_fmt *f) { if (f->msg_type != 42) { __builtin_trap(); } emits++; }
void AP_Logger::WriteBlock(const void *data, uint16_t n) {
    if (n>sizeof(record) || writes+1 != emits) { __builtin_trap(); }
    memcpy(record,data,n); last_length=n; writes++;
}
extern "C" int call(const uint8_t *in, unsigned ni, unsigned no) {
    return AP_Module::binding_log_write(in,ni,nullptr,no);
}
extern "C" void configure(unsigned m) { mode=m; calls=writes=emits=last_length=0; }
extern "C" unsigned registrations() { return calls; }
extern "C" unsigned write_count() { return writes; }
extern "C" unsigned read_record(uint8_t *out) { memcpy(out,record,last_length); return last_length; }
extern "C" const char *metadata(unsigned i) {
    const char *strings[] = {name_copy,fmt_copy,labels_copy,units_copy,mults_copy}; return strings[i];
}
''')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-DAP_MODULE_ENABLED=1', f'-DHAL_LOGGING_ENABLED={int(request.param)}',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(bindings.ROOT / 'BindingLog.cpp'),
                    str(source), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32]
    lib.read_record.argtypes = [ctypes.c_void_p]
    lib.metadata.restype = ctypes.c_char_p
    return lib, request.param


@pytest.mark.parametrize('code,payload', [
    (b'b', struct.pack('<b', -128)), (b'B', b'\xff'), (b'M', b'\x03'),
    (b'h', struct.pack('<h', -32768)), (b'c', struct.pack('<h', -100)),
    (b'H', struct.pack('<H', 65535)), (b'C', struct.pack('<H', 100)),
    (b'i', struct.pack('<i', -2147483648)), (b'e', struct.pack('<i', -100)),
    (b'L', struct.pack('<i', -353632610)), (b'I', struct.pack('<I', 4294967295)), (b'E', struct.pack('<I', 100)),
    (b'Q', struct.pack('<Q', 0xfedcba9876543210)), (b'f', struct.pack('<I', 0x7fc12345)),
    (b'n', b'a\0\xffx'), (b'N', bytes(range(16))), (b'Z', bytes(range(64))),
])
def test_exact_record_bytes(logger, code, payload):
    lib, enabled = logger
    data = packet(code, payload, units=(b'm', b'0'))
    assert lib.call(data, len(data), 0) == (0 if enabled else -3)
    out = ctypes.create_string_buffer(255)
    length = lib.read_record(out)
    expected = b'\xa3\x95\x2a' + struct.pack('<Q', 0x1122334455667788) + payload
    assert out.raw[:length] == (expected if enabled else b'')
    if enabled:
        assert [lib.metadata(i) for i in range(5)] == [b'RLOG', b'Q' + code, b'TimeUS,Value', b'sm', b'F0']
    assert lib.write_count() == (1 if enabled else 0)


def test_boundaries_and_invalid_packets(logger):
    lib, enabled = logger
    # 3*64 + 3*16 + 4 = 244, the largest safe payload after timestamp/header.
    cases = [packet(b'ZZZNNNi', bytes(244), labels=b'A,B,C,D,E,F,G'),
             packet(b'B' * 15, bytes(15), labels=b'A,B,C,D,E,F,G,H,I,J,K,L,M,N,O'),
             packet(labels=b'L' * 57)]
    for data in cases:
        assert lib.call(data, len(data), 0) == (0 if enabled else -3)
    lib.configure(0)
    invalid = [packet(name=b''), packet(name=b'12345'), packet(labels=b'L' * 58), packet(fmt=b'B' * 16),
               packet(fmt=b'd'), packet(fmt=b'q'), packet(fmt=b'a'), packet(fmt=b'g'), packet(fmt=b''),
               packet(labels=b'A,A'), packet(labels=b'TimeUS'), packet(labels=b',A'), packet(labels=b'A,'),
               packet(labels=b'A,B'), packet(units=(b'*', b'0')), packet(units=(b'm', b'*')),
               packet(units=(b'mm', b'0')), packet(units=(b'm', b'00')), packet(units=(b'', b'')),
               packet(b'ZZZNNNiB', bytes(245), labels=b'A,B,C,D,E,F,G,H')]
    for data in invalid:
        # All fixed-width boundary cases above retain the exact packet size,
        # except payload245, rejected by the first size check.
        assert lib.call(data, len(data), 0) == (-1 if enabled or len(data) != 358 else -3)
    for index, value in [(111, 2), (120, 1), (1, 0)]:
        data = bytearray(packet())
        data[index] = value
        assert lib.call(bytes(data), 358, 0) == (-1 if enabled else -3)
    for length in (245, 65535, 3, 5):
        data = bytearray(packet())
        data[112:114] = struct.pack('<H', length)
        assert lib.call(bytes(data), 358, 0) == (-1 if enabled else -3)
    data = bytearray(packet(units=(b'm', b'0')))
    data[111] = 0
    assert lib.call(bytes(data), 358, 0) == (-1 if enabled else -3)
    assert lib.call(packet(), 357, 0) == -1
    assert lib.call(packet(), 359, 0) == -1
    assert lib.call(packet(), 358, 1) == -1
    assert lib.call(None, 358, 0) == -1
    assert lib.registrations() == lib.write_count() == 0


def test_registration_failures_and_conflicts(logger):
    lib, enabled = logger
    for mode in range(1, 11):
        lib.configure(mode)
        data = packet(units=(b'm', b'0') if mode in (5, 8) else None)
        expected = -3 if not enabled or mode == 1 else -4 if mode == 2 else -1
        assert lib.call(data, 358, 0) == expected
        assert lib.registrations() == (1 if enabled and mode != 1 else 0)
        assert lib.write_count() == 0


def test_repeated_registration_reuses_type(logger):
    lib, enabled = logger
    for _ in range(2):
        assert lib.call(packet(), 358, 0) == (0 if enabled else -3)
        out = ctypes.create_string_buffer(255)
        length = lib.read_record(out)
        if enabled:
            assert length == 15 and out.raw[2] == 42
    assert lib.registrations() == lib.write_count() == (2 if enabled else 0)

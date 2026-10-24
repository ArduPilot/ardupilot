# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise the generated packet adapter with the host C-library parser."""

import ctypes
import math
import os
import struct
import subprocess

import bindings
import pytest
import strtof_binding


@pytest.fixture(scope='module', params=['-O0', '-Os'])
def parser(tmp_path_factory, request):
    work = tmp_path_factory.mktemp('strtof-binding')
    hal = work / 'AP_HAL'
    hal.mkdir()
    (hal / 'Semaphores.h').write_text('''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t); void give(); }; }
''')
    (hal / 'AP_HAL_Boards.h').write_text('''#pragma once
#define CONFIG_HAL_BOARD 99
#define HAL_BOARD_CHIBIOS 10
#define HAL_BOARD_SITL 3
''')
    lines = ['#include "BindingSupport.h"', '#include "HostApi.h"', '#include <stdlib.h>',
             'namespace AP_Module {']
    lines += strtof_binding.cpp(strtof_binding.specification(1))
    lines += ['}', '''extern "C" int call(const uint8_t *in, unsigned ni, uint8_t *out, unsigned no) {
    return AP_Module::binding_string_strtof(in, ni, out, no);
}''']
    source, binary = work / 'test.cpp', work / 'test.so'
    source.write_text('\n'.join(lines))
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', request.param,
                    '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-I' + str(work), '-I' + str(bindings.ROOT), '-I' + str(bindings.ROOT.parent),
                    str(source), '-o', str(binary)], check=True)
    lib = ctypes.CDLL(str(binary))
    lib.call.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.call.restype = ctypes.c_int32
    return lib


@pytest.mark.parametrize('text,value,consumed', [
    (b'', 0.0, 0), (b'junk', 0.0, 0), (b'  ', 0.0, 0),
    (b'25', 25.0, 2), (b'-0', -0.0, 2), (b'+.125E+2', 12.5, 8),
    (b'  -2.5tail', -2.5, 6), (b'1e+', 1.0, 1), (b'0x1.8p2', 6.0, 7),
    (b'1e9999', math.inf, 6), (b'-1e-9999', -0.0, 8),
    (b'NaN', math.nan, 3), (b'-inf', -math.inf, 4),
    (b'1.401298464324817e-45', 2**-149, 21),
    (b'0' * 255, 0.0, 255),
    (b'1' * 255, math.inf, 255),
    (b'0' * 252 + b'1.5', 1.5, 255),
])
def test_conversion(parser, text, value, consumed):
    packet = text.ljust(256, b'\0')
    incoming = ctypes.create_string_buffer(b'x' + packet + b'x', 258)
    output = ctypes.create_string_buffer(b'x' * 8, 8)
    assert parser.call(ctypes.byref(incoming, 1), 256, ctypes.byref(output, 1), 6) == 0
    assert incoming.raw == b'x' + packet + b'x'
    assert output.raw[:1] == output.raw[-1:] == b'x'
    actual, end = struct.unpack('<fH', output.raw[1:-1])
    assert end == consumed
    if math.isnan(value):
        assert math.isnan(actual)
    else:
        assert struct.pack('<f', actual) == struct.pack('<f', value)


def test_invalid_text_and_padding(parser):
    output = ctypes.create_string_buffer(b'!!!!!!', 6)
    for packet in (b'1' * 256, b'1\0junk'.ljust(256, b'\0')):
        assert parser.call(packet, 256, output, 6) == -1
    assert output.raw == b'!!!!!!'

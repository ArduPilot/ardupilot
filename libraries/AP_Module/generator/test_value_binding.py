# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Complete-value codecs, conservative copyback, and source layout sentinels."""

import copy
import ctypes
import os
import re
import struct
import subprocess
import xml.etree.ElementTree as ET

import bindings
import math_binding
import pytest
import singleton_binding
import value_binding


def test_complete_source_member_lists():
    # sizeof/offsetof cannot detect a new field filling padding. Include the
    # Location flag byte and MAVLink extension fields in source-level sentinels.
    source = (bindings.ROOT.parent / 'AP_Common/Location.h').read_text().split('enum class AltFrame')[0]
    members = re.findall(r'^\s*(\w+)\s+(\w+)\s*(?::\s*(\d+))?\s*;', source, re.M)
    assert [(n, 'boolean' if bits == '1' else t) for t, n, bits in members] == [
        (n, t) for n, t, _ in value_binding.TYPES['Location']['fields']]
    common = bindings.ROOT.parents[1] / 'modules/mavlink/message_definitions/v1.0/common.xml'
    root = ET.parse(common).getroot()
    message = root.find(".//message[@name='MISSION_ITEM_INT']")
    assert {f.attrib['name']: f.attrib['type'] for f in message.findall('field')} == {
        n: t for n, t, _ in value_binding.TYPES['mavlink_mission_item_int_t']['fields']}


def test_discovery_and_reject_incomplete_layout():
    ir = bindings.read_ir()
    owner = next(o for o in ir['userdata'] if o['name'] == 'Location')
    owner['methods'].append(dict(owner['methods'][0], name='new_value_method'))
    methods, reports = value_binding.select(ir, 1, bindings.method_contract)
    assert any(m['method'] == 'new_value_method' for m in methods)
    assert any(r['name'] == 'new_value_method' and r['status'] == 'generated' for r in reports)
    assert any(r['name'] == 'get_vector_from_origin_NEU' and r.get('reason') == 'deprecated' for r in reports)
    owner['fields'].append(dict(owner['fields'][0], name='unknown_field'))
    with pytest.raises(ValueError, match='changed value fields'):
        value_binding.select(ir, 1, bindings.method_contract)


@pytest.fixture(params=['-O0', '-O2'])
def codec(tmp_path, request):
    ir = bindings.read_ir()
    owner = next(o for o in ir['userdata'] if o['name'] == 'Location')
    methods, _ = value_binding.select(ir, 1, bindings.method_contract)
    methods = [m for m in methods if m['owner'] == 'Location' and
               m['method'] in ('set_lat', 'set_lng', 'set_alt', 'set_relative_alt', 'copy')]
    method = dict(copy.deepcopy(owner['methods'][0]), name='mutate_false', deprecate=None, rename=None, depends=None)
    method['return'] = math_binding.ir_type('boolean')
    method['arguments'] = [dict(math_binding.ir_type('float'), kind='userdata', name='Location'),
                           dict(math_binding.ir_type('Vector3f'), nullable=True)]
    method = bindings.method_contract(owner, method)
    method.update(id=500, value_receiver='Location', rust_name='mutate_false')
    methods.append(method)
    include = tmp_path / 'AP_HAL/Semaphores.h'
    include.parent.mkdir()
    include.write_text('''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t) { return true; } void give() {} }; }
''')
    lines = ['#include "BindingSupport.h"', '#include "HostApi.h"', '#include <stddef.h>',
             '#define LOCATION_ALT_MAX_M 83000', '#define AP_MISSION_ENABLED 1', '''
struct Vector3f { float x{}, y{}, z{}; };
struct Vector2f { float x{}, y{}; };
static unsigned calls;
struct Location {
    uint8_t relative_alt:1, loiter_ccw:1, terrain_alt:1, origin_alt:1, loiter_xtrack:1;
    int32_t alt{}, lat{}, lng{};
    Location() : relative_alt(0), loiter_ccw(0), terrain_alt(0), origin_alt(0), loiter_xtrack(0) {}
    bool mutate_false(Location &other, Vector3f &optional) {
        calls++; lat += 3; other.lng -= 7; optional.x = 99; return false;
    }
};
struct mavlink_mission_item_int_t {
    float param1{}, param2{}, param3{}, param4{};
    int32_t x{}, y{}; float z{}; uint16_t seq{}, command{};
    uint8_t target_system{}, target_component{}, frame{}, current{}, autocontinue{}, mission_type{};
};
namespace AP_Module {
''']
    names = sorted(value_binding.TYPES)
    lines += value_binding.cpp_helpers(set(names) | {'Vector3f'})
    for m in methods:
        lines += singleton_binding.cpp(m)
    lines += ['}', 'extern "C" int roundtrip(unsigned id, const uint8_t *in, uint8_t *out) {', 'switch(id) {']
    for i, name in enumerate(names):
        writer = ('AP_Module::BindingWire::write_native(out, value);' if name in value_binding.COMMON_WRITERS else
                  f'AP_Module::write_value_{name}(out, value);')
        lines += [f'case {i}: {{ {name} value{{}};',
                  f'if (in && !AP_Module::read_value_{name}(in, value)) {{ return -1; }}',
                  f'{writer} return 0; }}']
    lines += ['default: return -2; } }', '''
extern "C" int call(unsigned id, const uint8_t *in, unsigned ni, uint8_t *out, unsigned no) { switch(id) {''']
    lines += [f'case {m["id"]}: return AP_Module::binding_{m["id"]}(in, ni, out, no);' for m in methods]
    lines += ['default: return -2; } }', 'extern "C" unsigned call_count() { return calls; }']
    source, binary = tmp_path / 'value.cpp', tmp_path / 'value.so'
    source.write_text('\n'.join(lines))
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', request.param, '-Wall', '-Wextra', '-Werror',
                    '-fPIC', '-shared', '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source), '-o', str(binary)],
                   check=True)
    lib = ctypes.CDLL(str(binary))
    lib.roundtrip.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.c_void_p]
    lib.call.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    return lib, names, {m['method']: m for m in methods}


def location(lat=12, lng=-9):
    return bytes([1, 1, 0, 1, 0]) + struct.pack('<iii', 12345, lat, lng)


def test_all_fields_and_defaults(codec):
    lib, names, _ = codec
    for i, name in enumerate(names):
        n = value_binding.size(name)
        data = bytearray((j * 17 + 139) & 255 for j in range(n))
        if name == 'Location':
            data[:5] = [1, 0, 1, 0, 1]
        output = ctypes.create_string_buffer(n)
        assert lib.roundtrip(i, bytes(data), output) == 0
        assert output.raw == bytes(data)
        assert lib.roundtrip(i, None, output) == 0
        assert output.raw == bytes(n)
    output = ctypes.create_string_buffer(17)
    for field in range(5):
        data = bytearray(location())
        data[field] = 2
        assert lib.roundtrip(names.index('Location'), bytes(data), output) == -1


def test_copyback_on_false_and_field_assignment(codec):
    lib, _, methods = codec
    m = methods['mutate_false']
    ni, no = singleton_binding.sizes(m)
    output = ctypes.create_string_buffer(no)
    first, second = location(), location(33, 44)
    assert lib.call(m['id'], first + second, ni, output, no) == 0
    assert output.raw[:13] == bytes(13)  # false plus zero optional Vector3f
    assert output.raw[13:30] == location(15, -9)
    assert output.raw[30:] == location(33, 37)
    before = lib.call_count()
    for offset in (0, 17):
        for flag in range(5):
            invalid = bytearray(first + second)
            invalid[offset + flag] = 2
            output.raw = b'\xff' * no
            assert lib.call(m['id'], bytes(invalid), ni, output, no) == -1
            assert output.raw == bytes(no)
            assert lib.call_count() == before
    for method, valid, invalid in [('set_lat', 900000000, 900000001), ('set_lng', 1800000000, 1800000001),
                                   ('set_alt', 8299999, 8300000)]:
        m = methods[method]
        for v, status in [(valid, 0), (-valid, 0), (invalid, -1), (-invalid, -1)]:
            data = first + struct.pack('<i', v)
            output.raw = b'\xff' * no
            assert lib.call(m['id'], data, len(data), output, 17) == status
            if status == 0:
                assert output.raw[1] == 1  # hidden loiter_ccw survives
            else:
                assert output.raw[:17] == b'\xff' * 17
    m = methods['set_relative_alt']
    for flag, status in [(0, 0), (1, 0), (2, -1)]:
        output.raw = b'\xff' * no
        assert lib.call(m['id'], first + bytes([flag]), 18, output, 17) == status
        assert output.raw[:17] == (bytes([flag]) + first[1:] if status == 0 else b'\xff' * 17)
    # Out-of-range state from firmware is legal to copy/reuse, like Lua.
    m = methods['copy']
    invalid = location(1900000000)
    assert lib.call(m['id'], invalid, 17, output, 34) == 0
    assert output.raw[:34] == invalid * 2

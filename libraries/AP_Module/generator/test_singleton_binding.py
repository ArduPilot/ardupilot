# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise signature categories independently of concrete firmware methods."""

import copy
import ctypes
import json
import os
import struct
import subprocess

import bindings
import pytest
import singleton_binding as codec


def field(kind, **flags):
    return dict(kind='userdata' if kind in ('Vector3f', 'Quaternion') else kind,
                name=kind if kind in ('Vector3f', 'Quaternion') else None,
                **dict(dict(nullable=False, reference=False, pointer=False, skip_check=False, range=None), **flags))


def owner():
    return dict(name='Probe', rename='probe', depends='PROBE_ENABLED', get=None, literal=False,
                reference=False, semaphore=False, scheduler_semaphore=False, semaphore_pointer=False, aliases=[])


def enum_field(name, **flags):
    return dict(field('enum', **flags), name=name)


def literal(expression):
    return dict(field('literal'), name=expression)


def method(name, result, arguments):
    return dict(name=name, rename=None, depends=None, deprecate=None, line=1, return_=result, arguments=arguments)


def contract(owner, name, result, args, number):
    m = method(name, result, args)
    m['return'] = m.pop('return_')
    return dict(bindings.method_contract(owner, m), id=number)


@pytest.fixture(params=['none', 'semaphore', 'semaphore_pointer', 'scheduler_semaphore'])
def dispatch(tmp_path, request):
    o = owner()
    if request.param != 'none':
        o[request.param] = True
    methods = [contract(o, 'echo_' + name, field(name), [field(name, skip_check=name == 'float')], i + 1)
               for i, name in enumerate(codec.SCALARS)]
    methods += [
        contract(o, 'optional', field('boolean'), [field('uint8_t', range=['0', '2'])] +
                 [field(kind, nullable=True) for kind in ('int8_t', 'boolean', 'uint32_t', 'float', 'Vector3f')], 20),
        contract(o, 'refs', field('void'),
                 [field(kind, reference=True) for kind in ('int16_t', 'float', 'uint8_t')], 21),
        contract(o, 'consume', field('void'), [field('uint8_t', range=['0', '2'])], 22),
        contract(o, 'ranged', field('float'), [field('float', range=['-1', '1'])], 23),
        contract(o, 'dynamic', field('uint8_t'), [field('uint8_t', range=['0', 'ud->limit()'])], 24),
    ]
    methods.append(dict(methods[-1], id=25, depends='0'))
    # Exercise custom getter, literal pointer and literal reference access.
    for number, change in enumerate([dict(get='get_probe()'), dict(name='probe_pointer', literal=True),
                                     dict(name='get_probe_reference()', literal=True, reference=True)], 30):
        current = dict(o, **change)
        methods.append(contract(current, 'echo_int8_t', field('int8_t'), [field('int8_t')], number))
    enums = [('EU8', '-1', '300'), ('EU16', '0', '65535'), ('EU32', '0', 'INT32_MAX'),
             ('EI8', '-128', '127'), ('EI32', 'INT32_MIN', 'INT32_MAX'), ('Plain', 'PlainZero', 'PlainTwo')]
    for number, (name, low, high) in enumerate(enums, 40):
        methods.append(contract(o, 'echo_' + name, enum_field(name), [enum_field(name, range=[low, high])], number))
    methods += [contract(o, 'wide', enum_field('EU32'), [], 46),
                contract(o, 'literals', field('boolean'), [literal('false'),
                         enum_field('EU16', range=['EU16::Low', '((uint32_t)EU16::High-1)']), literal('3'),
                         field('uint16_t', nullable=True), literal('EU16::Low'), literal('"abc"')], 47)]
    methods += [dict(methods[-1], id=48, depends='0')]
    methods += [contract(o, 'strings', field('int32_t'),
                         [field('string'), field('int16_t'), literal('"%s"'), field('string')], 50)]
    methods += [dict(methods[-1], id=51, depends='0')]
    header = tmp_path / 'AP_HAL/Semaphores.h'
    header.parent.mkdir()
    header.write_text('''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL {
struct Semaphore {
    bool available = true;
    unsigned depth = 0;
    bool take(uint32_t ms) { if (ms != 0) { __builtin_trap(); } depth++; return true; }
    void give() { if (depth != 1) { __builtin_trap(); } depth--; }
};
}
''')
    cxx = ['#include "BindingSupport.h"', '#include "HostApi.h"',
           'enum class EU8 : uint8_t { Low=0 };', 'enum class EU16 : uint16_t { Low=0, High=4 };',
           'enum class EU32 : uint32_t { Low=0 };', 'enum class EI8 : int8_t { Low=-128 };',
           'enum class EI32 : int32_t { Low=INT32_MIN };', 'enum Plain { PlainZero=0, PlainTwo=2 };',
           'struct Vector3f { float x, y, z; };', 'struct Quaternion { float q1, q2, q3, q4; };',
           'static AP_HAL::Semaphore sem;', 'static unsigned calls, bounds;', 'static char copied[256];',
           'static bool present = true, have_semaphore = true, have_scheduler = true;',
           'struct Probe {', '    static Probe *get_singleton();']
    if request.param == 'semaphore_pointer':
        cxx.append('    AP_HAL::Semaphore *get_semaphore() { return have_semaphore ? &sem : nullptr; }')
    else:
        cxx.append('    AP_HAL::Semaphore &get_semaphore() { return sem; }')
    cxx += [f'    void check() {{ if (sem.depth != {int(request.param != "none")}) {{ __builtin_trap(); }} calls++; }}']
    for kind in codec.SCALARS:
        ctype = 'bool' if kind == 'boolean' else kind
        cxx.append(f'    {ctype} echo_{kind}({ctype} v) {{ check(); return v; }}')
    for name, _, _ in enums:
        cxx.append(f'    {name} echo_{name}({name} v) {{ check(); return v; }}')
    cxx += ['''    int32_t strings(const char *a, int16_t n, const char *format, const char *b) {
        check();
        if (strcmp(format, "%s")) { __builtin_trap(); }
        strcpy(copied, a);
        return int32_t(strlen(a)) + n + 1000 * int32_t(strlen(b));
    }
    EU32 wide() { check(); return static_cast<EU32>(UINT32_MAX); }
    bool literals(bool enabled, EU16 v, int fixed, uint16_t &out, EU16 zero, const char *str) {
        check();
        if (enabled || fixed != 3 || zero != EU16::Low || strcmp(str, "abc")) { __builtin_trap(); }
        out = uint16_t(v) + fixed;
        return v != EU16::Low;
    }''']
    cxx += ['''    bool optional(uint8_t valid, int8_t &a, bool &b, uint32_t &c, float &d, Vector3f &v) {
        check(); a = -128; b = true; c = UINT32_MAX; d = -0.0f; v = {1, 2, 3}; return valid != 0;
    }
    void refs(int16_t &a, float &b, uint8_t &c) { check(); a = INT16_MIN; b = INFINITY; c = UINT8_MAX; }
    void consume(uint8_t) { check(); }
    float ranged(float v) { check(); return v; }
    uint8_t dynamic(uint8_t v) { check(); return v; }
    uint8_t limit() { check(); bounds++; return 2; }
};
static Probe probe;
static Probe *probe_pointer = &probe;
Probe *Probe::get_singleton() { return present ? &probe : nullptr; }
static Probe *get_probe() { return Probe::get_singleton(); }
static Probe &get_probe_reference() { return probe; }
struct AP_Scheduler {
    static AP_Scheduler *get_singleton() { static AP_Scheduler s; return have_scheduler ? &s : nullptr; }
    AP_HAL::Semaphore &get_semaphore() { return sem; }
};
namespace AP_Module {
''']
    for m in methods:
        cxx += codec.cpp(m)
    cxx += ['}', 'extern "C" int32_t call(uint32_t id, const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {',
            '    switch (id) {']
    cxx += [f'    case {m["id"]}: return AP_Module::binding_{m["id"]}(in, ni, out, no);' for m in methods]
    cxx += ['    default: return -1;', '    }', '}', '''
extern "C" unsigned count() { return calls; }
extern "C" unsigned copied_byte(unsigned i) { return uint8_t(copied[i]); }
extern "C" unsigned bound_count() { return bounds; }
extern "C" unsigned held() { return sem.depth; }
extern "C" void configure(bool exists, bool available, bool semaphore, bool scheduler) {
    present = exists; sem.available = available; have_semaphore = semaphore; have_scheduler = scheduler;
    probe_pointer = exists ? &probe : nullptr;
}
''']
    source, library = tmp_path / 'probe.cpp', tmp_path / 'probe.so'
    source.write_text('\n'.join(cxx))
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-DPROBE_ENABLED=1', '-DAP_SCHEDULER_ENABLED=1', '-fPIC', '-shared',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.call.restype = ctypes.c_int32
    lib.configure.argtypes = [ctypes.c_bool] * 4
    return lib, {m['method']: m['id'] for m in methods if m['id'] < 25}, request.param


@pytest.mark.parametrize('kind,format_,values', [
    ('boolean', '?', [False, True]), ('uint8_t', 'B', [0, 255]), ('int8_t', 'b', [-128, 127]),
    ('uint16_t', 'H', [0, 65535]), ('int16_t', 'h', [-32768, 32767]),
    ('uint32_t', 'I', [0, 0xffffffff]), ('int32_t', 'i', [-0x80000000, 0x7fffffff]),
    ('float', 'f', [-0.0, -1.25, float('inf'), float('-inf'), float('nan')]),
])
def test_scalar_roundtrip(dispatch, kind, format_, values):
    lib, ids, _ = dispatch
    for value in values:
        data = struct.pack('<' + format_, value)
        out = ctypes.create_string_buffer(b'x' * (len(data) + 2), len(data) + 2)
        assert lib.call(ids['echo_' + kind], data, len(data), ctypes.byref(out, 1), len(data)) == 0
        assert out.raw == b'x' + data + b'x'
        assert lib.held() == 0
    if kind == 'boolean':
        assert lib.call(ids['echo_boolean'], b'\2', 1, out, 1) == -1


def test_multiple_outputs_and_void(dispatch):
    lib, ids, _ = dispatch
    out = ctypes.create_string_buffer(23)
    assert lib.call(ids['optional'], b'\1', 1, out, 23) == 0
    assert out.raw == b'\1' + struct.pack('<b?Iffff', -128, True, 0xffffffff, -0.0, 1, 2, 3)
    assert lib.call(ids['optional'], b'\0', 1, out, 23) == 0
    assert out.raw == bytes(23)
    assert lib.call(ids['optional'], b'\3', 1, out, 23) == -1
    assert lib.call(ids['refs'], None, 0, out, 7) == 0
    assert out.raw[:7] == struct.pack('<hfB', -32768, float('inf'), 255)
    assert lib.call(ids['consume'], b'\1', 1, None, 0) == 0


def test_ranges_and_access(dispatch):
    lib, ids, lock = dispatch
    out = ctypes.create_string_buffer(4)
    for value in (-1.0, 0.0, 1.0):
        assert lib.call(ids['ranged'], struct.pack('<f', value), 4, out, 4) == 0
    for value in (-1.01, 1.01, float('nan'), float('inf')):
        before = lib.count()
        assert lib.call(ids['ranged'], struct.pack('<f', value), 4, out, 4) == -1
        assert lib.count() == before
    assert lib.call(ids['dynamic'], b'\2', 1, out, 1) == 0
    assert lib.call(ids['dynamic'], b'\3', 1, out, 1) == -1
    assert lib.bound_count() == 2 and lib.held() == 0
    before = lib.count()
    assert lib.call(25, b'\1', 1, out, 1) == -3
    assert lib.count() == before
    for id_ in (30, 31, 32):
        assert lib.call(id_, b'\x80', 1, out, 1) == 0
        assert out.raw[0] == 128
    lib.configure(False, True, True, True)
    for id_ in (ids['dynamic'], 30, 31):
        assert lib.call(id_, b'\1', 1, out, 1) == -3
    lib.configure(True, False, True, True)
    before = lib.bound_count()
    assert lib.call(ids['dynamic'], b'\1', 1, out, 1) == 0
    assert lib.bound_count() == before + 1
    lib.configure(True, True, False, False)
    status = -3 if lock in ('semaphore_pointer', 'scheduler_semaphore') else 0
    assert lib.call(ids['dynamic'], b'\1', 1, out, 1) == status
    assert lib.held() == 0


def test_discovery_and_coverage():
    ir = bindings.read_ir()
    selection = json.loads((bindings.ROOT / 'generator/bindings.json').read_text())
    spec = bindings.select(ir, selection)
    reports = [r for r in spec['coverage']['entries'] if r['kind'] == 'singleton-method']
    generated_fields = sum(r['kind'] == 'singleton-field' and r['status'] == 'generated'
                           for r in spec['coverage']['entries'])
    standalone_adapters = sum(r['status'] == 'generated' and
                              r.get('reason') in ('owned-battery-state-adapter',
                                                  'module-lifecycle-owned-adapter',
                                                  'revalidated-efi-backend-adapter',
                                                  'revalidated-scripting-backend-adapter',
                                                  'owned-filesystem-stat-adapter',
                                                  'owned-telemetry-value-adapter',
                                                  'serialized-persistent-motor-configuration-adapter',
                                                  'module-channel-claimed-persistent-driver-adapter',
                                                  'single-module-persistent-client-adapter',
                                                  'owned-applet-adapter',
                                                  'module-lifecycle-owned-crsf-menu-adapter',
                                                  'owned-camera-value-adapter',
                                                  'module-lifecycle-owned-custom-mode-adapter',
                                                  'module-lifecycle-owned-analog-pwm-adapter',
                                                  'owned-efi-state-adapter',
                                                  'native-rust-deprecated-compatibility-wrapper',
                                                  'native-rust-ipv4-formatting',
                                                  'guarded-filesystem-format-adapter',
                                                  'owned-u64-state-adapter')
                              for r in reports)
    assert sum(not (m.get('receiver') or m.get('value_receiver')) for m in spec['methods']) == (
        sum(r['status'] == 'generated' for r in reports) + generated_fields - standalone_adapters)
    assert len(reports) == sum(len(o['methods']) for o in ir['singletons'])
    assert all(r['status'] == 'generated' or r['reason'] for r in reports)
    # New ordinary methods are discovered without adding a selection entry.
    changed = copy.deepcopy(ir)
    o = next(o for o in changed['singletons'] if o['name'] == 'AP_AHRS')
    m = copy.deepcopy(next(m for m in o['methods'] if m['name'] == 'get_hagl'))
    m.update(name='new_estimate', rename='public_estimate')
    m['arguments'].append(field('int16_t', nullable=True))
    o['methods'].append(m)
    expanded = bindings.select(changed, selection)
    assert len(expanded['methods']) == len(spec['methods']) + 1
    assert expanded['coverage'] != spec['coverage']
    added = next(m for m in expanded['methods'] if m['api_name'] == 'ahrs.public_estimate')
    assert 'Result<Option<(f32, i16)>>' in '\n'.join(codec.rust(added))
    # Parser declaration order never controls IDs or coverage ordering.
    changed['singletons'].reverse()
    for o in changed['singletons']:
        o['methods'].reverse()
    assert bindings.select(changed, selection) == expanded


def test_enum_and_literal_categories(dispatch):
    lib, _, lock = dispatch
    out = ctypes.create_string_buffer(4)
    for id_, low, high in [(40, 0, 255), (41, 0, 65535), (42, 0, 0x7fffffff),
                           (43, -128, 127), (44, -0x80000000, 0x7fffffff), (45, 0, 2)]:
        for value in {low, high, 0}:
            assert lib.call(id_, struct.pack('<i', value), 4, out, 4) == 0
            assert struct.unpack('<i', out.raw)[0] == value
        for value in {low - 1, high + 1, -0x80000000, 0x7fffffff}:
            if not -0x80000000 <= value <= 0x7fffffff or low <= value <= high:
                continue
            before = lib.count()
            assert lib.call(id_, struct.pack('<i', value), 4, out, 4) == -1
            assert lib.count() == before
    assert lib.call(46, None, 0, out, 4) == -4  # enum result outside signed wire domain
    for value in (0, 1, 3):
        assert lib.call(47, struct.pack('<i', value), 4, out, 3) == 0
        assert out.raw[:3] == (bytes(3) if value == 0 else struct.pack('<BH', 1, value + 3))
    assert lib.call(47, struct.pack('<i', 4), 4, out, 3) == -1
    assert lib.call(48, struct.pack('<i', 1), 4, out, 3) == -3
    lib.configure(False, True, True, True)
    assert lib.call(47, struct.pack('<i', 1), 4, out, 3) == -3
    lib.configure(True, False, True, True)
    assert lib.call(47, struct.pack('<i', 1), 4, out, 3) == 0


@pytest.mark.parametrize('value', ['x()', 'x; y', '"a\\n"', '"a""b"', 'x + 1', ''])
def test_unsupported_literals(value):
    with pytest.raises(ValueError, match='unsupported-literal'):
        codec.field(literal(value), argument=True)


@pytest.mark.parametrize('value', [
    'ud', 'value', 'lock', 'scheduler', 'owner', 'token', 'input', 'input_size',
    'output', 'output_size', 'arg0', 'arg123', 'bits2', 'ud::Member',
])
def test_literal_local_capture(value):
    with pytest.raises(ValueError, match='literal-captures-local'):
        codec.field(literal(value), argument=True)


@pytest.mark.parametrize('flag', ['nullable', 'reference'])
def test_enum_reference_is_unspecified(flag):
    with pytest.raises(ValueError, match='enum-output-reference-unspecified'):
        codec.field(enum_field('E', **{flag: True}), argument=True)


def test_enum_and_literal_source_shapes():
    m = contract(owner(), 'choose', enum_field('First::Mode'),
                 [literal('false'), enum_field('Other::Mode', range=['0', '3']), literal('3')], 1)
    rust = '\n'.join(codec.rust(m))
    assert 'arg0: crate::enums::OtherMode' in rust
    assert 'Result<crate::enums::FirstMode>' in rust
    assert 'arg1' not in rust and 'arg2' not in rust
    assert codec.sizes(m) == (4, 4)
    assert 'ud->choose(false, static_cast<Other::Mode>(arg1), 3)' in '\n'.join(codec.cpp(m))
    assert 'pub struct FirstMode(pub i32)' in codec.rust_enums([m], '')
    m['result']['enum'] = 'Other_Mode'
    with pytest.raises(ValueError, match='duplicate Rust enum'):
        codec.rust_enums([m], '')


def test_mav_severity_constants_from_xml(tmp_path):
    m = contract(owner(), 'severity', enum_field('MAV_SEVERITY'), [], 1)
    generated = codec.rust_enums([m], '')
    for value, name in enumerate(('EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 'WARNING', 'NOTICE', 'INFO', 'DEBUG')):
        assert f'pub const {name}: Self = Self({value});' in generated
    assert 'pub struct MavSeverity(pub i32)' in generated

    xml = tmp_path / 'common.xml'
    xml.write_text('''<mavlink><enums><enum name="MAV_SEVERITY">
        <entry name="MAV_SEVERITY_INFO" value="42"/>
        </enum></enums></mavlink>''')
    generated = codec.rust_enums([m], '', mavlink_xml=xml)
    assert 'pub const INFO: Self = Self(42);' in generated
    assert 'pub const INFO: Self = Self(6);' not in generated


@pytest.mark.parametrize('body', [
    '',
    '<enum name="MAV_SEVERITY"/>',
    '<enum name="MAV_SEVERITY"/><enum name="MAV_SEVERITY"/>',
    '<enum name="MAV_SEVERITY"><entry name="OTHER_INFO" value="6"/></enum>',
    '<enum name="MAV_SEVERITY"><entry name="MAV_SEVERITY_bad" value="6"/></enum>',
    '<enum name="MAV_SEVERITY"><entry name="MAV_SEVERITY_INFO" value="2147483648"/></enum>',
    '<enum name="MAV_SEVERITY"><entry name="MAV_SEVERITY_INFO" value="6"/>'
    '<entry name="MAV_SEVERITY_INFO" value="7"/></enum>',
])
def test_mav_severity_rejects_invalid_definitions(tmp_path, body):
    xml = tmp_path / 'common.xml'
    xml.write_text('<mavlink><enums>' + body + '</enums></mavlink>')
    m = contract(owner(), 'severity', enum_field('MAV_SEVERITY'), [], 1)
    with pytest.raises(ValueError, match='MAVLink enum'):
        codec.rust_enums([m], '', mavlink_xml=xml)


@pytest.mark.parametrize('a,b', [
    (b'', b''), (b'x', b'x' * 254), (b'percent %s %n', b'other'),
    (b'a' * 255, b'b' * 255), ('\u00e9'.encode(), b'end'),
])
def test_borrowed_strings(dispatch, a, b):
    lib, _, lock = dispatch
    packet = a.ljust(256, b'\0') + struct.pack('<h', -123) + b.ljust(256, b'\0')
    out = ctypes.create_string_buffer(4)
    assert lib.call(50, packet, len(packet), out, 4) == 0
    assert struct.unpack('<i', out.raw)[0] == len(a) - 123 + 1000 * len(b)
    mutable = ctypes.create_string_buffer(packet, len(packet))
    assert lib.call(50, mutable, len(packet), out, 4) == 0
    mutable.raw = bytes(len(packet))
    assert bytes(lib.copied_byte(i) for i in range(len(a) + 1)) == a + b'\0'
    for invalid in (b'x' * 256 + packet[256:], packet[:258] + b'x' * 256,
                    b'\0x' + packet[2:], packet[:258] + b'\0x' + packet[260:]):
        before = lib.count()
        assert lib.call(50, invalid, len(invalid), out, 4) == -1
        assert lib.count() == before
    assert lib.call(51, packet, len(packet), out, 4) == -3
    lib.configure(False, True, True, True)
    assert lib.call(50, packet, len(packet), out, 4) == -3
    lib.configure(True, False, True, True)
    assert lib.call(50, packet, len(packet), out, 4) == 0


def test_string_signature_policy():
    for value, argument in [(field('string'), False), (field('string', nullable=True), True),
                            (field('string', reference=True), True)]:
        with pytest.raises(ValueError, match='string-output-ownership'):
            codec.field(value, argument=argument)
    assert codec.field(field('string'), argument=True)['type'] == 'cstring'
    assert codec.field(field('string'), argument=True, parameter=True)['type'] == 'string'


def test_header_order_and_policies(monkeypatch):
    ir = bindings.read_ir()
    selection = json.loads((bindings.ROOT / 'generator/bindings.json').read_text())
    spec = bindings.select(ir, selection)
    headers = spec['headers']
    substitutions = bindings.policies()['headers']
    expected = []
    for h in ir['includes']:
        name = substitutions.get(h['name'], {}).get('replace', h['name'])
        if name:
            expected.append(dict(h, name=name))
    assert headers == expected
    policy = bindings.policies()
    policy['excluded']['Gone.method'] = 'persistent-state'
    monkeypatch.setattr(bindings, 'policies', lambda: policy)
    with pytest.raises(ValueError, match='stale'):
        bindings.select(ir, selection)


def test_custom_accessors_require_review(monkeypatch):
    ir = bindings.read_ir()
    selection = json.loads((bindings.ROOT / 'generator/bindings.json').read_text())
    policy = bindings.policies()
    del policy['accessors']['AP_OSD_Backend']
    monkeypatch.setattr(bindings, 'policies', lambda: policy)
    spec = bindings.select(ir, selection)
    reports = [r for r in spec['coverage']['entries'] if r['owner'] == 'AP_OSD_Backend']
    assert reports and all(r['reason'] == 'unaudited-accessor' for r in reports)
    policy['accessors']['Gone'] = {}
    with pytest.raises(ValueError, match='stale accessor'):
        bindings.select(ir, selection)


def test_changed_accessor_fails_closed():
    ir = bindings.read_ir()
    selection = json.loads((bindings.ROOT / 'generator/bindings.json').read_text())
    owner = next(o for o in ir['singletons'] if o['name'] == 'AP_OSD_Backend')
    owner['get'] = 'AP_OSD::different_accessor()'
    with pytest.raises(ValueError, match='changed or unexplained accessor'):
        bindings.select(ir, selection)

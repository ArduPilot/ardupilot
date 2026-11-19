# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate the bounded fence polygon and notify flag adapters."""

import copy
import ctypes
import json
import os
import struct
import subprocess

import bindings
import fence_binding as fence
import generate
import notify_binding as notify
import pytest

STUBS = r'''#pragma once
#include <stdint.h>
#include <string.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL {
struct Semaphore {
    unsigned depth = 0;
    bool take(uint32_t) { depth++; return true; }
    void give() { depth--; }
};
}
inline AP_HAL::Semaphore scheduler_sem;
class AP_Scheduler {
public:
    static AP_Scheduler *get_singleton() { static AP_Scheduler s; return &s; }
    AP_HAL::Semaphore &get_semaphore() { return scheduler_sem; }
};
struct Vector2l { int32_t x, y; };
enum class AC_PolyFenceType : uint8_t { POLYGON_INCLUSION = 98 };
class AC_PolyFenceItem {
public:
    AC_PolyFenceType type;
    Vector2l loc;
    uint8_t vertex_count;
    float radius;
};
inline bool check_lat(int32_t lat) { return lat >= -900000000 && lat <= 900000000; }
inline bool check_lng(int32_t lng) { return lng >= -1800000000 && lng <= 1800000000; }
inline AC_PolyFenceItem written[32];
inline unsigned written_count, write_calls, locked_writes;
class AC_PolyFence_loader {
public:
    bool write_fence(const AC_PolyFenceItem *items, uint16_t count) {
        write_calls++;
        locked_writes += scheduler_sem.depth == 1;
        memcpy(written, items, count * sizeof(items[0]));
        written_count = count;
        return items[0].loc.x != 1;
    }
};
class AC_Fence {
public:
    static AC_Fence *get_singleton() { static AC_Fence f; return &f; }
    AC_PolyFence_loader &polyfence() { return loader; }
    AC_PolyFence_loader loader;
};
struct NotifyFlags {
'''


def notify_fields():
    return ''.join(f'    bool {name};\n' for name in notify.FLAGS) + '};\n'


@pytest.fixture(scope='module')
def host(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp('fence_notify')
    outputs = generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text()))
    for name in ('HostApi.h', 'BindingConfig.h'):
        path = tmp_path / 'AP_Module' / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(outputs[name])
    (tmp_path / 'AP_HAL').mkdir()
    (tmp_path / 'AP_HAL/Semaphores.h').write_text('#include "../stubs.h"\n')
    (tmp_path / 'stubs.h').write_text(STUBS + notify_fields() + '''
#define AP_NOTIFY_EVENT_COUNTS_ENABLED 1
class AP_Notify {
public:
    static inline NotifyFlags flags;
    // deliberately not in contract order
    struct notify_events_type {
        uint32_t arming_failed : 1;
        uint32_t padding : 5;
        uint32_t temp_cal_failed : 1;
        uint32_t temp_cal_saved : 1;
        uint32_t initiated_temp_cal : 1;
        uint32_t compass_cal_canceled : 1;
        uint32_t compass_cal_failed : 1;
        uint32_t compass_cal_saved : 1;
        uint32_t initiated_compass_cal : 1;
        uint32_t rest : 19;
    };
    static inline uint8_t event_counts[32];
    static uint8_t get_event_count(uint8_t bit) { return event_counts[bit]; }
};
''')
    lines = ['#include "stubs.h"', '#include <AP_Module/HostApi.h>', '#include <AP_Module/BindingSupport.h>',
             '#define AP_FENCE_ENABLED 1', '#define AP_SCHEDULER_ENABLED 1', 'namespace AP_Module {']
    lines += fence.cpp(fence.specification(10))
    lines += notify.cpp(notify.specification(11))
    lines += ['}',
              'extern "C" int fence_call(const void *in, void *out) {',
              ' return AP_Module::binding_fence_write_inclusion_polygon((const uint8_t *)in, 169, (uint8_t *)out, 1); }',
              'extern "C" int notify_call(void *out) {',
              ' return AP_Module::binding_notify_get_flags(nullptr, 0, (uint8_t *)out, 4); }',
              'extern "C" void set_flag(unsigned bit, bool value) { ((bool *)&AP_Notify::flags)[bit] = value; }',
              'extern "C" int events_call(void *out) {',
              ' return AP_Module::binding_notify_get_event_counts(nullptr, 0, (uint8_t *)out, 8); }',
              'extern "C" void set_event_count(unsigned bit, unsigned count) { AP_Notify::event_counts[bit] = count; }',
              'extern "C" unsigned calls() { return write_calls; }',
              'extern "C" unsigned locked() { return locked_writes; }',
              'extern "C" unsigned held() { return scheduler_sem.depth; }',
              'extern "C" unsigned count() { return written_count; }',
              'extern "C" int item(unsigned i, unsigned field) {',
              ' const auto &w = written[i];',
              ' return field == 0 ? int(w.type) : field == 1 ? w.loc.x : field == 2 ? w.loc.y : w.vertex_count; }']
    source = tmp_path / 'host.cpp'
    source.write_text('\n'.join(lines) + '\n')
    library = tmp_path / 'host.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT.parent), str(source), '-o', str(library)],
                   check=True)
    lib = ctypes.CDLL(str(library))
    lib.fence_call.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.notify_call.argtypes = [ctypes.c_void_p]
    lib.notify_call.restype = ctypes.c_int
    lib.events_call.argtypes = [ctypes.c_void_p]
    lib.events_call.restype = ctypes.c_int
    return lib


def polygon(points, count=None):
    data = b''.join(struct.pack('<ii', lat, lng) for lat, lng in points)
    return bytes([len(points) if count is None else count]) + data.ljust(8 * fence.POINTS_MAX, b'\0')


def test_selects_audited_owners_only():
    ir = bindings.read_ir()
    assert fence.select(ir, 5) == fence.specification(5)
    assert notify.select(ir, 6)['methods'] == notify.specification(6)['methods']
    for module, owner_name in ((fence, 'AC_Fence'), (notify, 'AP_Notify')):
        changed = copy.deepcopy(ir)
        owner = next(o for o in changed['singletons'] if o['name'] == owner_name)
        owner['scheduler_semaphore'] = True
        with pytest.raises(ValueError, match='unaudited'):
            module.select(changed, 5)


def test_contracts_are_bounded():
    method = fence.specification(5)['methods'][0]
    assert method['arguments'] == [dict(type='u8,range3-21', name='count'),
                                   dict(type='i32-le[42]', name='lat_lng')]
    assert 'blocking-scheduler-semaphore' in method['policy']
    assert notify.FLAGS[:3] == ('armed', 'flying', 'have_pos_abs')
    assert len(set(notify.FLAGS)) == len(notify.FLAGS) <= 32
    names = [m['name'] for m in bindings.specification()['fence']['methods']]
    assert names == ['fence.write_inclusion_polygon']


def test_fence_polygon_is_checked_and_locked(host):
    one = ctypes.create_string_buffer(1)
    points = [(-353632620, 1491652370), (-353632620, 1491660000), (-353640000, 1491660000)]
    assert host.fence_call(polygon(points), one) == 0 and one.raw == b'\1'
    assert host.calls() == host.locked() == 1 and host.held() == 0
    assert host.count() == 3
    for i, (lat, lng) in enumerate(points):
        assert [host.item(i, f) for f in range(4)] == [98, lat, lng, 3]
    # A loader rejection is a false result, not an error.
    assert host.fence_call(polygon([(1, 2)] * 3), one) == 0 and one.raw == b'\0'
    before = host.calls()
    bad = [polygon(points[:2]), polygon(points * 8, 22), polygon(points, 0),
           polygon(points, 2), polygon([(900000001, 0)] * 3), polygon([(0, -1800000001)] * 3)]
    tail = bytearray(polygon(points))
    tail[1 + 8 * 3] = 1
    for packet in bad + [bytes(tail)]:
        one.raw = b'x'
        assert host.fence_call(packet, one) == -1 and one.raw == b'\0'
    assert host.calls() == before and host.held() == 0


def test_notify_flag_bits_follow_contract_order(host):
    out = ctypes.create_string_buffer(4)
    assert host.notify_call(out) == 0 and out.raw == bytes(4)
    for bit in range(len(notify.FLAGS)):
        host.set_flag(bit, True)
        assert host.notify_call(out) == 0
        assert struct.unpack('<I', out.raw)[0] == 1 << bit
        host.set_flag(bit, False)
    rust = notify.rust(notify.specification(9), '')
    for bit, name in enumerate(notify.FLAGS):
        assert f'pub const {name.upper()}: u32 = 1 << {bit};' in rust


def test_notify_event_counts_follow_contract_order(host):
    out = ctypes.create_string_buffer(8)
    assert host.events_call(out) == 0 and out.raw == bytes(8)
    # bit positions of the stub's notify_events_type
    bits = dict(arming_failed=0, temp_cal_failed=6, temp_cal_saved=7, initiated_temp_cal=8,
                compass_cal_canceled=9, compass_cal_failed=10, compass_cal_saved=11, initiated_compass_cal=12)
    for byte, name in enumerate(notify.EVENTS):
        host.set_event_count(bits[name], 200 + byte)
        assert host.events_call(out) == 0
        assert out.raw == bytes(byte) + bytes([200 + byte]) + bytes(7 - byte)
        host.set_event_count(bits[name], 0)
    assert len(set(notify.EVENTS)) == len(notify.EVENTS) == 8
    rust = notify.rust(notify.specification(9), '')
    for byte, name in enumerate(notify.EVENTS):
        assert f'pub const {name.upper()}: u32 = {8 * byte};' in rust

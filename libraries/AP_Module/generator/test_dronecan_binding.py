# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Audit module-owned DroneCAN subscription and broadcast handles."""

import copy
import ctypes
import os
import struct
import subprocess
import sys

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import bindings
import dronecan_binding
import pytest


def test_contract_is_bounded_and_context_owned():
    spec = dronecan_binding.specification(20)
    assert [method['id'] for method in spec['methods']] == [20, 21, 22, 23, 24, 25, 26]
    assert [method['name'] for method in spec['methods']] == [
        'DroneCan.subscribe',
        'DroneCanSubscription.check_message',
        'DroneCanSubscription.close',
        'DroneCan.open',
        'DroneCanHandle.broadcast',
        'DroneCanHandle.close',
        'DroneCanHandle.request',
    ]
    assert 'cleanup-after-worker-join' in spec['methods'][0]['policy']
    assert 'per-handle-transfer-id' in spec['methods'][4]['policy']
    assert spec['methods'][1]['result']['payload'] == 'bytes-max256-zero-pad256'
    assert spec['methods'][1]['contract_aliases'] == ['DroneCanHandle.check_message']
    assert spec['methods'][4]['arguments'][1]['type'] == 'bytes-max256'
    assert spec['methods'][6]['arguments'][1]['range'] == [1, 127]


def test_sdk_does_not_expose_pointer_or_copyable_resource():
    spec = dronecan_binding.specification(20)
    rust = dronecan_binding.rust(spec, '')
    declaration = rust.split('pub struct DroneCanSubscription', 1)[0]
    assert 'Copy' not in declaration
    assert '*mut' not in rust and '*const' not in rust
    assert 'context.raw' in rust
    assert rust.count('pub fn close(self') == 2
    assert 'pub fn broadcast(&self' in rust
    assert 'pub fn request(&self' in rust
    assert rust.count('pub fn check_message(&self') == 2
    assert f'let mut output = [0u8; {dronecan_binding.MESSAGE_WIRE_SIZE}]' in rust


@pytest.fixture(params=[True, False])
def dronecan_host(tmp_path, request):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <mutex>
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
#define CONCAT_(a,b) a##b
#define CONCAT(a,b) CONCAT_(a,b)
#define WITH_SEMAPHORE(s) AP_HAL::Semaphore::Guard CONCAT(_guard_, __LINE__)(s)
namespace AP_HAL { class Semaphore { public:
    bool take(uint32_t) { mutex.lock(); return true; } void take_blocking() { mutex.lock(); } void give() { mutex.unlock(); }
    class Guard { public: explicit Guard(Semaphore &s): sem(s) { sem.take_blocking(); } ~Guard() { sem.give(); }
        private: Semaphore &sem; };
    private: std::recursive_mutex mutex;
}; }
using HAL_Semaphore = AP_HAL::Semaphore;
''',
        'AP_HAL/AP_HAL.h': '''#pragma once
#include <AP_HAL/Semaphores.h>
#include <new>
#define NEW_NOTHROW new (std::nothrow)
''',
        'AP_Math/AP_Math.h': '#pragma once\n#define MIN(a,b) ((a) < (b) ? (a) : (b))\n',
        'AP_Scripting/AP_Scripting_config.h': '#pragma once\n',
        'AP_HAL/utility/RingBuffer.h': '''#pragma once
#include <deque>
#include <stddef.h>
extern bool reject_payload_push;
template <typename T> class ObjectBuffer { public:
    explicit ObjectBuffer(size_t capacity): capacity(capacity) {}
    size_t space() const { return capacity - values.size(); }
    bool push(const T &value) {
        if (reject_payload_push || space() == 0) { return false; }
        values.push_back(value); return true;
    }
    bool pop(T &value) { if (values.empty()) { return false; } value=values.front(); values.pop_front(); return true; }
private: size_t capacity; std::deque<T> values;
};
''',
        'AP_DroneCAN/AP_DroneCAN.h': '''#pragma once
#include <AP_HAL/AP_HAL.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#define CANARD_IFACE_ALL 0xFF
enum CanardTransferType { CanardTransferTypeBroadcast, CanardTransferTypeRequest, CanardTransferTypeResponse };
struct CanardRxTransfer { uint16_t payload_len; uint8_t source_node_id; uint64_t timestamp_usec;
    bool canfd; const uint8_t *payload; CanardTransferType transfer_type; uint8_t transfer_id; };
inline int canardDecodeScalar(const CanardRxTransfer *transfer, uint32_t offset, uint8_t, bool, void *out) {
    memcpy(out, transfer->payload + offset / 8, 1); return 0;
}
namespace Canard {
struct Transfer { CanardTransferType transfer_type; uint64_t data_type_signature; uint16_t data_type_id;
    uint8_t *inout_transfer_id; uint8_t priority; const void *payload; uint32_t payload_len;
    uint8_t iface_mask; bool canfd; uint32_t timeout_ms; };
class HandlerList { public:
    HandlerList(CanardTransferType, uint16_t, uint64_t, uint8_t) {}
    virtual ~HandlerList() {}
    void link() { if (!linked) { linked=true; alive++; latest=this; } }
    void unlink() { if (linked) { linked=false; alive--; if (latest==this) { latest=nullptr; } } }
    static unsigned alive;
    static HandlerList *latest;
    bool deliver(const CanardRxTransfer &transfer) { return handle_message(transfer); }
private:
    virtual bool handle_message(const CanardRxTransfer &) = 0;
    bool linked=false;
};
}
class FakeCanardInterface { public:
    bool broadcast(const Canard::Transfer &transfer) {
        signature=transfer.data_type_signature; data_type=transfer.data_type_id;
        transfer_id=*transfer.inout_transfer_id; priority=transfer.priority; iface_mask=transfer.iface_mask;
        canfd=transfer.canfd; timeout_ms=transfer.timeout_ms; count++;
        payload.assign(static_cast<const uint8_t *>(transfer.payload),
                       static_cast<const uint8_t *>(transfer.payload) + transfer.payload_len);
        if (succeed) { *transfer.inout_transfer_id=uint8_t((*transfer.inout_transfer_id + 1U) & 31U); }
        return succeed;
    }
    bool request(uint8_t target, const Canard::Transfer &transfer) {
        signature=transfer.data_type_signature; data_type=transfer.data_type_id;
        transfer_id=*transfer.inout_transfer_id; priority=transfer.priority; iface_mask=transfer.iface_mask;
        canfd=transfer.canfd; timeout_ms=transfer.timeout_ms; destination=target;
        transfer_type=transfer.transfer_type; count++;
        payload.assign(static_cast<const uint8_t *>(transfer.payload),
                       static_cast<const uint8_t *>(transfer.payload) + transfer.payload_len);
        if (succeed) { *transfer.inout_transfer_id=uint8_t((*transfer.inout_transfer_id + 1U) & 31U); }
        return succeed;
    }
    HAL_Semaphore &get_sem_rx() { return rx; }
    HAL_Semaphore rx; bool succeed=true; uint64_t signature=0; uint16_t data_type=0;
    uint8_t transfer_id=0, priority=0, iface_mask=0, destination=0; bool canfd=false;
    uint32_t timeout_ms=0, count=0; CanardTransferType transfer_type=CanardTransferTypeBroadcast;
    std::vector<uint8_t> payload;
};
class AP_DroneCAN { public:
    explicit AP_DroneCAN(uint8_t index): index(index) {}
    static AP_DroneCAN *get_dronecan(uint8_t index);
    FakeCanardInterface &get_canard_iface() { return iface; }
    uint8_t get_driver_index() const { return index; }
private: uint8_t index; FakeCanardInterface iface;
};
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source, library = tmp_path / 'dronecan.cpp', tmp_path / 'dronecan.so'
    source.write_text('''#include <AP_DroneCAN/AP_DroneCAN.h>
#include "BindingDroneCAN.h"
unsigned Canard::HandlerList::alive;
Canard::HandlerList *Canard::HandlerList::latest;
bool reject_payload_push;
#if TEST_WRAP_PAYLOAD
static int payload_allocations;
extern "C" void *__real_malloc(size_t);
extern "C" void __real_free(void *);
extern "C" void *__wrap_malloc(size_t size) {
    void *result = __real_malloc(size);
    if (result != nullptr) { payload_allocations++; }
    return result;
}
extern "C" void __wrap_free(void *value) {
    if (value != nullptr) { payload_allocations--; }
    __real_free(value);
}
extern "C" int allocations() { return payload_allocations; }
#endif
extern "C" bool receive(bool reject) {
    reject_payload_push=reject;
    const uint8_t payload[] = {1, 2, 3};
    const CanardRxTransfer transfer{3, 42, 123456, false, payload, CanardTransferTypeBroadcast, 0};
    return Canard::HandlerList::latest->deliver(transfer);
}
static AP_DroneCAN drivers[2] = { AP_DroneCAN(0), AP_DroneCAN(1) };
static bool present[2] = {true, true};
AP_DroneCAN *AP_DroneCAN::get_dronecan(uint8_t index) { return index < 2 && present[index] ? &drivers[index] : nullptr; }
extern "C" int call(unsigned id, void *ctx, const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    switch (id) {
    case 0: return AP_Module::binding_dronecan_subscribe(ctx,in,ni,out,no);
    case 1: return AP_Module::binding_dronecan_check_message(ctx,in,ni,out,no);
    case 2: return AP_Module::binding_dronecan_close(ctx,in,ni,out,no);
    case 3: return AP_Module::binding_dronecan_open(ctx,in,ni,out,no);
    case 4: return AP_Module::binding_dronecan_broadcast(ctx,in,ni,out,no);
    case 5: return AP_Module::binding_dronecan_close(ctx,in,ni,out,no);
    case 6: return AP_Module::binding_dronecan_request(ctx,in,ni,out,no);
    default: return -1;
    }
}
extern "C" void cleanup(void *ctx) { AP_Module::binding_dronecan_cleanup(ctx); }
extern "C" void set_next_token(uint32_t token) { AP_Module::binding_dronecan_test_set_next_token(token); }
extern "C" void configure(unsigned driver, bool available, bool succeed) {
    present[driver]=available; drivers[driver].get_canard_iface().succeed=succeed;
}
extern "C" uint64_t seen(unsigned driver, unsigned field) {
    auto &iface=drivers[driver].get_canard_iface();
    switch (field) {
    case 0: return iface.signature; case 1: return iface.data_type; case 2: return iface.transfer_id;
    case 3: return iface.priority; case 4: return iface.iface_mask; case 5: return iface.canfd;
    case 6: return iface.timeout_ms; case 7: return iface.payload.size(); case 8: return iface.count;
    case 10: return iface.destination; case 11: return iface.transfer_type;
    default: return Canard::HandlerList::alive;
    }
}
extern "C" uint8_t seen_payload(unsigned driver, unsigned index) {
    return drivers[driver].get_canard_iface().payload[index];
}
''')
    enabled = int(request.param)
    subprocess.run([
        os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
        '-DAP_MODULE_ENABLED=1', f'-DHAL_ENABLE_DRONECAN_DRIVERS={enabled}',
        '-DCANARD_ENABLE_CANFD=1', '-DAP_MODULE_BINDING_TESTING=1',
        '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
        str(source), str(bindings.ROOT / 'BindingDroneCAN.cpp'),
        *(['-DTEST_WRAP_PAYLOAD=1', '-Wl,--wrap=malloc,--wrap=free'] if sys.platform.startswith('linux')
          else ['-DTEST_WRAP_PAYLOAD=0']), '-o', str(library),
    ], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32,
                         ctypes.c_void_p, ctypes.c_uint32]
    lib.call.restype = ctypes.c_int
    lib.cleanup.argtypes = [ctypes.c_void_p]
    lib.set_next_token.argtypes = [ctypes.c_uint32]
    lib.configure.argtypes = [ctypes.c_uint, ctypes.c_bool, ctypes.c_bool]
    lib.seen.argtypes = [ctypes.c_uint, ctypes.c_uint]
    lib.seen.restype = ctypes.c_uint64
    lib.seen_payload.argtypes = [ctypes.c_uint, ctypes.c_uint]
    lib.seen_payload.restype = ctypes.c_uint8
    lib.receive.argtypes = [ctypes.c_bool]
    lib.receive.restype = ctypes.c_bool
    return lib, request.param


def open_handle(lib, context, driver=0, signature=0x1122334455667788, data_type=123, can_fd=False):
    packet = bytes([driver]) + struct.pack('<QH', signature, data_type) + bytes([can_fd])
    output = ctypes.create_string_buffer(4)
    status = lib.call(3, context, packet, len(packet), output, len(output))
    return status, output.raw


def send_broadcast(lib, context, token, payload):
    packet = token + struct.pack('<H', len(payload)) + payload
    output = ctypes.create_string_buffer(1)
    status = lib.call(4, context, packet, len(packet), output, 1)
    return status, output.raw


def send_request(lib, context, token, target_node, payload):
    packet = token + bytes([target_node]) + struct.pack('<H', len(payload)) + payload
    output = ctypes.create_string_buffer(1)
    status = lib.call(6, context, packet, len(packet), output, 1)
    return status, output.raw


@pytest.mark.skipif(not sys.platform.startswith('linux'), reason='allocation tracking requires GNU ld --wrap')
def test_receive_releases_payload_when_queue_rejects_push(dronecan_host):
    lib, enabled = dronecan_host
    if not enabled:
        return
    context = ctypes.c_void_p(24)
    packet = b'\0' + struct.pack('<QH', 0x99, 44)
    token = ctypes.create_string_buffer(4)
    assert lib.call(0, context, packet, len(packet), token, 4) == 0
    assert lib.allocations() == 0
    assert lib.receive(True)
    assert lib.allocations() == 0
    output = ctypes.create_string_buffer(dronecan_binding.MESSAGE_WIRE_SIZE)
    assert lib.call(1, context, token, 4, output, len(output)) == 0
    assert output.raw == bytes(len(output))
    assert lib.receive(False)
    assert lib.allocations() == 1
    assert lib.call(1, context, token, 4, output, len(output)) == 0
    assert output.raw[:6] == b'\1\3\0\1\2\3'
    assert lib.allocations() == 0
    assert lib.receive(False)
    assert lib.allocations() == 1
    lib.cleanup(context)
    assert lib.allocations() == 0


def test_broadcast_preserves_transfer_metadata_and_id(dronecan_host):
    lib, enabled = dronecan_host
    context = ctypes.c_void_p(10)
    status, token = open_handle(lib, context, driver=1, can_fd=True)
    assert status == (0 if enabled else -3)
    assert token != bytes(4) if enabled else token == bytes(4)
    if not enabled:
        return

    payload = b'Battery'
    assert send_broadcast(lib, context, token, payload) == (0, b'\1')
    assert [lib.seen(1, field) for field in range(8)] == [
        0x1122334455667788, 123, 0, 0, 0xFF, 1, 10, len(payload)]
    assert bytes(lib.seen_payload(1, index) for index in range(len(payload))) == payload
    assert send_broadcast(lib, context, token, payload) == (0, b'\1')
    assert lib.seen(1, 2) == 1
    lib.configure(1, True, False)
    assert send_broadcast(lib, context, token, payload) == (0, b'\0')
    assert lib.seen(1, 2) == 2
    assert send_broadcast(lib, context, token, payload) == (0, b'\0')
    assert lib.seen(1, 2) == 2
    lib.cleanup(context)


def test_request_installs_response_subscription_and_preserves_metadata(dronecan_host):
    lib, enabled = dronecan_host
    context = ctypes.c_void_p(15)
    status, token = open_handle(lib, context, driver=1, can_fd=True)
    if not enabled:
        assert status == -3 and token == bytes(4)
        return
    assert status == 0 and lib.seen(1, 9) == 0
    payload = b'parameter'
    assert send_request(lib, context, token, 42, payload) == (0, b'\1')
    assert [lib.seen(1, field) for field in (0, 1, 2, 3, 4, 5, 6, 7, 10, 11)] == [
        0x1122334455667788, 123, 0, 0, 0xFF, 1, 10, len(payload), 42, 1]
    assert bytes(lib.seen_payload(1, index) for index in range(len(payload))) == payload
    assert lib.seen(1, 9) == 1
    assert send_request(lib, context, token, 42, payload) == (0, b'\1')
    assert lib.seen(1, 2) == 1 and lib.seen(1, 9) == 1
    lib.configure(1, True, False)
    assert send_request(lib, context, token, 42, payload) == (0, b'\0')
    assert lib.seen(1, 9) == 0
    lib.cleanup(context)
    assert lib.seen(1, 9) == 0


def test_request_wire_bounds(dronecan_host):
    lib, enabled = dronecan_host
    context = ctypes.c_void_p(16)
    status, token = open_handle(lib, context)
    if not enabled:
        assert status == -3
        return
    assert send_request(lib, context, token, 0, b'x')[0] == -1
    assert send_request(lib, context, token, 128, b'x')[0] == -1
    assert send_request(lib, context, token, 127, bytes(dronecan_binding.PAYLOAD_MAX)) == (0, b'\1')
    payload = bytes(dronecan_binding.PAYLOAD_MAX + 1)
    assert send_request(lib, context, token, 1, payload)[0] == -1
    malformed = token + b'\1' + struct.pack('<H', 2) + b'x'
    assert lib.call(6, context, malformed, len(malformed), ctypes.create_string_buffer(1), 1) == -1
    lib.cleanup(context)


def test_broadcast_bounds_context_and_lifecycle(dronecan_host):
    lib, enabled = dronecan_host
    context = ctypes.c_void_p(20)
    status, token = open_handle(lib, context)
    if not enabled:
        assert status == -3 and token == bytes(4)
        return
    assert status == 0 and token != bytes(4)
    assert send_broadcast(lib, context, token, bytes(dronecan_binding.PAYLOAD_MAX)) == (0, b'\1')
    payload = bytes(dronecan_binding.PAYLOAD_MAX + 1)
    packet = token + struct.pack('<H', len(payload)) + payload
    assert lib.call(4, context, packet, len(packet), ctypes.create_string_buffer(1), 1) == -1
    malformed = token + struct.pack('<H', 2) + b'x'
    assert lib.call(4, context, malformed, len(malformed), ctypes.create_string_buffer(1), 1) == -1
    assert send_broadcast(lib, ctypes.c_void_p(21), token, b'x')[0] == -1

    lib.cleanup(context)
    assert send_broadcast(lib, context, token, b'x')[0] == -1
    status, restarted = open_handle(lib, context)
    assert status == 0 and restarted not in (bytes(4), token)
    assert send_broadcast(lib, context, restarted, b'x') == (0, b'\1')
    assert lib.call(5, context, restarted, 4, None, 0) == 0


def test_shared_handle_limit_and_subscription_cleanup(dronecan_host):
    lib, enabled = dronecan_host
    if not enabled:
        return
    handles = []
    for index in range(16):
        context = ctypes.c_void_p(100 + index)
        status, token = open_handle(lib, context)
        assert status == 0 and token != bytes(4)
        handles.append((context, token))
    assert open_handle(lib, ctypes.c_void_p(999))[0] == -2
    lib.cleanup(handles[0][0])

    subscribe = b'\0' + struct.pack('<QH', 0x99, 44)
    output = ctypes.create_string_buffer(4)
    assert lib.call(0, handles[0][0], subscribe, len(subscribe), output, 4) == 0
    assert output.raw != bytes(4) and lib.seen(0, 9) == 1
    lib.cleanup(handles[0][0])
    assert lib.seen(0, 9) == 0
    for context, _ in handles[1:]:
        lib.cleanup(context)


def test_cleanup_serializes_with_broadcast(dronecan_host):
    lib, enabled = dronecan_host
    if not enabled:
        return
    context = ctypes.c_void_p(300)
    status, token = open_handle(lib, context)
    assert status == 0

    def send_many():
        return [send_broadcast(lib, context, token, b'1234567')[0] for _ in range(100)]

    with ThreadPoolExecutor(max_workers=4) as pool:
        calls = [pool.submit(send_many) for _ in range(3)]
        cleanup = pool.submit(lib.cleanup, context)
        statuses = [status for call in calls for status in call.result()]
        cleanup.result()
    assert all(status in (0, -1) for status in statuses)
    assert send_broadcast(lib, context, token, b'x')[0] == -1


def test_token_wrap_skips_zero_and_live_collision(dronecan_host):
    lib, enabled = dronecan_host
    if not enabled:
        return
    context = ctypes.c_void_p(350)
    status, first = open_handle(lib, context)
    assert status == 0 and first == struct.pack('<I', 1)
    lib.set_next_token(0xFFFFFFFF)
    status, maximum = open_handle(lib, context)
    assert status == 0 and maximum == struct.pack('<I', 0xFFFFFFFF)
    status, wrapped = open_handle(lib, context)
    assert status == 0 and wrapped == struct.pack('<I', 2)
    assert len({first, maximum, wrapped}) == 3
    for token in (first, maximum, wrapped):
        assert send_broadcast(lib, context, token, b'x') == (0, b'\1')
    lib.cleanup(context)


def test_open_wire_validation(dronecan_host):
    lib, _ = dronecan_host
    context = ctypes.c_void_p(400)
    packet = b'\0' + struct.pack('<QH', 1, 2) + b'\2'
    output = ctypes.create_string_buffer(4)
    assert lib.call(3, context, packet, len(packet), output, 4) == -1
    assert lib.call(3, context, packet[:-1], len(packet) - 1, output, 4) == -1
    assert lib.call(3, None, packet, len(packet), output, 4) == -1


def test_ir_selection_fails_closed():
    ir = bindings.read_ir()
    assert dronecan_binding.select(ir, 20)['methods'][4]['name'] == 'DroneCanHandle.broadcast'
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['userdata'] if item['name'] == 'DroneCAN_Handle')
    owner['creation_args'] = 3
    with pytest.raises(ValueError, match='metadata'):
        dronecan_binding.select(changed, 20)
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['userdata'] if item['name'] == 'DroneCAN_Handle')
    owner['methods'][0]['return']['kind'] = 'uint8_t'
    with pytest.raises(ValueError, match='subscribe'):
        dronecan_binding.select(changed, 20)
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['userdata'] if item['name'] == 'DroneCAN_Handle')
    next(alias for alias in owner['aliases'] if alias['alias'] == 'broadcast')['manual'] = False
    with pytest.raises(ValueError, match='manual operations'):
        dronecan_binding.select(changed, 20)


def test_request_coverage_and_dispatch():
    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    request = coverage[('DroneCAN_Handle', 'request')]
    assert request['status'] == 'generated'
    assert request['api'] == 'DroneCanHandle.request'
    assert 'return binding_dronecan_request' in bindings.emit(spec)['Bindings_generated.cpp']


def test_generated_rust_compiles(tmp_path):
    rust = dronecan_binding.rust(dronecan_binding.specification(20), '')
    source = tmp_path / 'dronecan.rs'
    source.write_text('''#![no_std]
#[derive(Clone, Copy)] pub enum Error { Invalid, Host }
pub type Result<T=()> = core::result::Result<T, Error>;
pub fn result(value: i32) -> Result { if value == 0 { Ok(()) } else { Err(Error::Host) } }
pub struct Api { pub binding_call: Option<unsafe extern "C" fn(
    *mut core::ffi::c_void, u32, *const u8, u32, *mut u8, u32) -> i32> }
pub struct Context<'a> { raw: *mut core::ffi::c_void, api: &'a Api }
pub mod bindings {
    pub struct Dronecan<'context, 'api> {
        pub(crate) context: &'context crate::Context<'api>,
    }
}
impl<'api> Context<'api> {
    pub fn dronecan(&self) -> bindings::Dronecan<'_, 'api> {
        bindings::Dronecan { context: self }
    }
}
pub mod imports { pub struct Import; impl Import { pub const fn slot(&self)->u32 { 0 } }
''' + ''.join(f'pub static BINDING_{index}: Import = Import;\n' for index in range(20, 27)) + '''}
mod generated {
''' + rust + '\n}\n')
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--crate-type', 'lib', '--edition', '2021',
                    str(source), '-o', str(tmp_path / 'dronecan.rlib')], check=True)


def test_runtime_has_slot_context_and_post_join_cleanup():
    runtime = (Path(__file__).parents[1] / 'Runtime.cpp').read_text()
    assert '{&slot, binding_dispatch, binding_resolve}' in runtime
    cleanup = runtime.index('binding_dronecan_cleanup(&slot);')
    release = runtime.index('Module::release(slot.module);', cleanup)
    assert cleanup < release

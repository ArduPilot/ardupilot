# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise module-owned raw CAN buffers, frame copying, and cleanup races."""

import copy
import ctypes
import os
import struct
import subprocess

from concurrent.futures import ThreadPoolExecutor

import bindings
import can_binding
import pytest


@pytest.fixture(params=[True, False])
def can_host(tmp_path, request):
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
    bool take(uint32_t) { mutex.lock(); return true; } void give() { mutex.unlock(); }
    class Guard { public: explicit Guard(Semaphore &s): sem(s) { sem.take(0); } ~Guard() { sem.give(); }
        private: Semaphore &sem; };
    private: std::recursive_mutex mutex;
}; }
using HAL_Semaphore = AP_HAL::Semaphore;
''',
        'AP_HAL/AP_HAL.h': '''#pragma once
#include <AP_HAL/Semaphores.h>
#include <stdint.h>
#include <string.h>
#define HAL_CANFD_SUPPORTED 1
namespace AP_HAL { struct CANFrame {
    static const uint8_t NonFDCANMaxDataLen=8; uint32_t id=0; uint8_t data[64]{}; uint8_t dlc=0; bool canfd=false;
    static uint8_t dlcToDataLength(uint8_t value) { static const uint8_t sizes[16] =
        {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64}; return sizes[value > 15 ? 15 : value]; }
}; }
''',
        'AP_Scripting/AP_Scripting_CANSensor.h': '''#pragma once
#include <AP_HAL/AP_HAL.h>
#include <deque>
#include <vector>
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
class ScriptingCANSensor;
class ScriptingCANBuffer { public:
    explicit ScriptingCANBuffer(ScriptingCANSensor &owner): sensor(owner) {}
    bool write_frame(AP_HAL::CANFrame &frame, uint32_t timeout);
    bool read_frame(AP_HAL::CANFrame &frame) { std::lock_guard<std::mutex> lock(mutex);
        if (frames.empty()) { return false; } frame=frames.front(); frames.pop_front(); return true; }
    bool add_filter(uint32_t mask, uint32_t value) { std::lock_guard<std::mutex> lock(mutex);
        if (filters.size() == 8) { return false; } filters.push_back({mask,value&mask}); return true; }
    void inject(const AP_HAL::CANFrame &frame) { std::lock_guard<std::mutex> lock(mutex); frames.push_back(frame); }
    ScriptingCANSensor &sensor; std::mutex mutex; std::deque<AP_HAL::CANFrame> frames;
    std::vector<std::pair<uint32_t,uint32_t>> filters;
};
class ScriptingCANSensor { public:
    bool initialized() const { return ready; }
    ScriptingCANBuffer *add_buffer(uint32_t) { auto *buffer=new ScriptingCANBuffer(*this);
        std::lock_guard<std::mutex> lock(mutex); buffers.push_back(buffer); return buffer; }
    bool remove_buffer(ScriptingCANBuffer *buffer) { std::lock_guard<std::mutex> lock(mutex);
        for (auto it=buffers.begin(); it != buffers.end(); ++it) if (*it == buffer) {
            buffers.erase(it); delete buffer; return true; } return false; }
    void inject(const AP_HAL::CANFrame &frame) { std::lock_guard<std::mutex> lock(mutex);
        for (auto *buffer: buffers) buffer->inject(frame); }
    bool ready=true; std::mutex mutex; std::vector<ScriptingCANBuffer*> buffers;
    AP_HAL::CANFrame last{}; uint32_t last_timeout=0;
};
inline bool ScriptingCANBuffer::write_frame(AP_HAL::CANFrame &frame, uint32_t timeout) {
    sensor.last=frame; sensor.last_timeout=timeout; return true;
}
ScriptingCANSensor *get_scripting_can_sensor(uint8_t index);
#endif
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    source, library = tmp_path / 'can.cpp', tmp_path / 'can.so'
    source.write_text('''#include <AP_Scripting/AP_Scripting_CANSensor.h>
#include "BindingCAN.h"
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
static ScriptingCANSensor sensors[2];
ScriptingCANSensor *get_scripting_can_sensor(uint8_t index) { return index < 2 ? &sensors[index] : nullptr; }
#endif
extern "C" int call(unsigned id, void *ctx, const uint8_t *in, uint32_t ni, uint8_t *out, uint32_t no) {
    switch (id) {
    case 0: return AP_Module::binding_can_get_device(ctx,in,ni,out,no);
    case 1: return AP_Module::binding_can_get_device2(ctx,in,ni,out,no);
    case 2: return AP_Module::binding_can_read_frame(ctx,in,ni,out,no);
    case 3: return AP_Module::binding_can_write_frame(ctx,in,ni,out,no);
    case 4: return AP_Module::binding_can_add_filter(ctx,in,ni,out,no);
    case 5: return AP_Module::binding_can_close(ctx,in,ni,out,no);
    default: return -1;
    }
}
extern "C" void cleanup(void *ctx) { AP_Module::binding_can_cleanup(ctx); }
extern "C" void set_next_token(uint32_t token) { AP_Module::binding_can_test_set_next_token(token); }
extern "C" unsigned alive(unsigned index) {
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    return sensors[index].buffers.size();
#else
    (void)index; return 0;
#endif
}
extern "C" void inject(unsigned index, uint32_t id, uint8_t dlc, bool fd) {
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    AP_HAL::CANFrame frame; frame.id=id; frame.dlc=dlc; frame.canfd=fd;
    for (unsigned i=0; i<sizeof(frame.data); i++) { frame.data[i]=uint8_t(i+1); }
    sensors[index].inject(frame);
#else
    (void)index; (void)id; (void)dlc; (void)fd;
#endif
}
extern "C" uint32_t last_id(unsigned index) {
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    return sensors[index].last.id;
#else
    (void)index; return 0;
#endif
}
extern "C" uint8_t last_dlc(unsigned index) {
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    return sensors[index].last.dlc;
#else
    (void)index; return 0;
#endif
}
extern "C" uint8_t last_data(unsigned index, unsigned offset) {
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    return sensors[index].last.data[offset];
#else
    (void)index; (void)offset; return 0;
#endif
}
extern "C" bool last_fd(unsigned index) {
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    return sensors[index].last.canfd;
#else
    (void)index; return false;
#endif
}
extern "C" uint32_t last_timeout(unsigned index) {
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    return sensors[index].last_timeout;
#else
    (void)index; return 0;
#endif
}
''')
    enabled = int(request.param)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-DAP_MODULE_ENABLED=1', '-DAP_MODULE_BINDING_TESTING=1',
                    f'-DAP_SCRIPTING_CAN_SENSOR_ENABLED={enabled}',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
                    str(source), str(bindings.ROOT / 'BindingCAN.cpp'), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32,
                         ctypes.c_void_p, ctypes.c_uint32]
    lib.call.restype = ctypes.c_int
    lib.cleanup.argtypes = [ctypes.c_void_p]
    lib.set_next_token.argtypes = [ctypes.c_uint32]
    lib.inject.argtypes = [ctypes.c_uint, ctypes.c_uint32, ctypes.c_uint8, ctypes.c_bool]
    for name in ('alive', 'last_id', 'last_dlc', 'last_data', 'last_fd', 'last_timeout'):
        getattr(lib, name).restype = ctypes.c_uint32
    return lib, request.param


def allocate(lib, context, bus=0, length=5):
    output = ctypes.create_string_buffer(4)
    status = lib.call(bus, context, bytes([length]), 1, output, 4)
    return status, output.raw


def test_lifecycle_is_bounded_and_context_owned(can_host):
    lib, enabled = can_host
    if not enabled:
        status, token = allocate(lib, ctypes.c_void_p(1))
        assert status == -3 and token == bytes(4)
        return
    for length in (0, can_binding.BUFFER_LENGTH_MAX + 1):
        status, token = allocate(lib, ctypes.c_void_p(200), length=length)
        assert status == -1 and token == bytes(4)
    contexts = [ctypes.c_void_p(index + 1) for index in range(can_binding.BUFFER_COUNT_MAX)]
    tokens = []
    for context in contexts:
        status, token = allocate(lib, context)
        assert status == 0 and token != bytes(4)
        tokens.append(token)
    assert lib.alive(0) == can_binding.BUFFER_COUNT_MAX
    status, token = allocate(lib, ctypes.c_void_p(100))
    assert status == -2 and token == bytes(4)
    output = ctypes.create_string_buffer(1)
    assert lib.call(4, contexts[1], tokens[0] + bytes(8), 12, output, 1) == -1
    lib.cleanup(contexts[0])
    assert lib.alive(0) == can_binding.BUFFER_COUNT_MAX - 1
    assert lib.call(2, contexts[0], tokens[0], 4, ctypes.create_string_buffer(71), 71) == -1
    for context in contexts[1:]:
        lib.cleanup(context)
    assert lib.alive(0) == 0
    status, token = allocate(lib, contexts[0])
    assert status == 0 and lib.alive(0) == 1
    assert lib.call(5, contexts[0], token, 4, None, 0) == 0
    assert lib.alive(0) == 0


def test_frame_copy_filter_limit_and_validation(can_host):
    lib, enabled = can_host
    if not enabled:
        return
    context = ctypes.c_void_p(55)
    status, token = allocate(lib, context, bus=1)
    assert status == 0
    output = ctypes.create_string_buffer(1)
    for index in range(can_binding.FILTER_COUNT_MAX + 1):
        packet = token + struct.pack('<II', 0x1fffffff, index)
        assert lib.call(4, context, packet, len(packet), output, 1) == 0
        assert output.raw == bytes([index < can_binding.FILTER_COUNT_MAX])

    lib.inject(1, 0x81234567, 15, True)
    frame = ctypes.create_string_buffer(can_binding.FRAME_WIRE_SIZE + 1)
    assert lib.call(2, context, token, 4, frame, len(frame)) == 0
    assert frame.raw[:7] == b'\1' + struct.pack('<IBB', 0x81234567, 15, 1)
    assert frame.raw[7:] == bytes(range(1, 65))

    write = token + struct.pack('<IBB', 0x123, 8, 0) + bytes(range(64)) + struct.pack('<I', 10000)
    assert lib.call(3, context, write, len(write), output, 1) == 0 and output.raw == b'\1'
    assert lib.last_id(1) == 0x123 and lib.last_dlc(1) == 8 and not lib.last_fd(1)
    assert lib.last_data(1, 7) == 7 and lib.last_timeout(1) == 10000
    max_timeout = write[:-4] + struct.pack('<I', can_binding.WRITE_TIMEOUT_MAX_US)
    assert lib.call(3, context, max_timeout, len(max_timeout), output, 1) == 0
    assert lib.last_timeout(1) == can_binding.WRITE_TIMEOUT_MAX_US
    too_long = write[:-4] + struct.pack('<I', can_binding.WRITE_TIMEOUT_MAX_US + 1)
    assert lib.call(3, context, too_long, len(too_long), output, 1) == -1
    invalid = bytearray(write)
    invalid[8] = 9
    assert lib.call(3, context, bytes(invalid), len(invalid), output, 1) == -1
    invalid[8], invalid[9] = 8, 2
    assert lib.call(3, context, bytes(invalid), len(invalid), output, 1) == -1
    lib.cleanup(context)


def test_cleanup_serializes_with_calls(can_host):
    lib, enabled = can_host
    if not enabled:
        return
    context = ctypes.c_void_p(77)
    status, token = allocate(lib, context)
    assert status == 0

    def read_many():
        output = ctypes.create_string_buffer(can_binding.FRAME_WIRE_SIZE + 1)
        return [lib.call(2, context, token, 4, output, len(output)) for _ in range(200)]

    with ThreadPoolExecutor(max_workers=4) as pool:
        futures = [pool.submit(read_many) for _ in range(3)]
        cleanup = pool.submit(lib.cleanup, context)
        results = [status for future in futures for status in future.result()]
        cleanup.result()
    assert all(status in (0, -1) for status in results)
    assert lib.alive(0) == 0


def test_token_wrap_skips_zero_and_live_collision(can_host):
    lib, enabled = can_host
    if not enabled:
        return
    context = ctypes.c_void_p(88)
    status, first = allocate(lib, context)
    assert status == 0 and first == struct.pack('<I', 1)
    lib.set_next_token(0xFFFFFFFF)
    status, maximum = allocate(lib, context)
    assert status == 0 and maximum == struct.pack('<I', 0xFFFFFFFF)
    status, wrapped = allocate(lib, context)
    assert status == 0 and wrapped == struct.pack('<I', 2)
    assert len({first, maximum, wrapped}) == 3
    lib.cleanup(context)


def test_ir_selection_fails_closed():
    ir = bindings.read_ir()
    assert can_binding.select(ir, 50)['methods'][0]['name'] == 'CAN.get_device'
    mutations = []
    changed = copy.deepcopy(ir)
    next(item for item in changed['singletons'] if item['name'] == can_binding.OWNER)['depends'] = None
    mutations.append((changed, 'owner metadata'))
    changed = copy.deepcopy(ir)
    frame = next(item for item in changed['userdata'] if item['name'] == can_binding.FRAME)
    next(field for field in frame['fields'] if field['name'] == 'dlc')['type']['range'][1] = '8'
    mutations.append((changed, 'frame fields'))
    changed = copy.deepcopy(ir)
    frame = next(item for item in changed['userdata'] if item['name'] == can_binding.FRAME)
    frame['methods'][0]['return']['kind'] = 'uint8_t'
    mutations.append((changed, 'frame methods'))
    changed = copy.deepcopy(ir)
    buffer = next(item for item in changed['ap_objects'] if item['name'] == can_binding.BUFFER)
    buffer['methods'][0]['arguments'][1]['skip_check'] = False
    mutations.append((changed, 'buffer methods'))
    for changed, error in mutations:
        with pytest.raises(ValueError, match=error):
            can_binding.select(changed, 50)


def test_generated_rust_compiles(tmp_path):
    rust = can_binding.rust(can_binding.specification(50), '')
    assert "impl crate::bindings::Can<'_, '_>" in rust
    assert 'pub fn get_device(&self, buffer_length: u8)' in rust
    assert 'pub fn get_device2(&self, buffer_length: u8)' in rust
    source = tmp_path / 'can.rs'
    source.write_text('''#![no_std]
#[derive(Clone, Copy)] pub enum Error { Invalid, Host }
pub type Result<T=()> = core::result::Result<T, Error>;
pub fn result(value: i32) -> Result { if value == 0 { Ok(()) } else { Err(Error::Host) } }
pub struct Api { pub binding_call: Option<unsafe extern "C" fn(
    *mut core::ffi::c_void, u32, *const u8, u32, *mut u8, u32) -> i32> }
pub struct Context<'a> { raw: *mut core::ffi::c_void, api: &'a Api }
pub mod bindings {
    use super::Context;
    pub struct Can<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
pub mod imports { pub struct Import; impl Import { pub const fn slot(&self)->u32 { 0 } }
''' + ''.join(f'pub static BINDING_{index}: Import = Import;\n' for index in range(50, 56)) +
                      '}\nmod generated {\n' + rust + '\n}\n')
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--crate-type', 'lib', '--edition', '2021',
                    str(source), '-o', str(tmp_path / 'can.rlib')], check=True)


def test_runtime_cleanup_is_after_join():
    runtime = (bindings.ROOT / 'Runtime.cpp').read_text()
    joined = runtime.index('if (!joined)')
    cleanup = runtime.index('binding_can_cleanup(&slot);')
    release = runtime.index('Module::release(slot.module);', cleanup)
    assert joined < cleanup < release
    scripting = bindings.ROOT.parent / 'AP_Scripting'
    lua = (scripting / 'lua_bindings.cpp').read_text()
    sensor = (scripting / 'AP_Scripting_CANSensor.cpp').read_text()
    assert lua.count('get_scripting_can_sensor(') == 2
    assert 'NEW_NOTHROW ScriptingCANSensor' not in lua
    assert 'ScriptingCANSensor::remove_buffer' in sensor

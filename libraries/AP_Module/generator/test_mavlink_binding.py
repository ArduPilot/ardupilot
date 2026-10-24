# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate the owned MAVLink generator contract, SDK wire and host service."""

import copy
import ctypes
import os
import subprocess

from pathlib import Path

import bindings
import mavlink_binding as mavlink
import pytest


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert mavlink.select(ir, 500) == mavlink.specification(500)
    for alias, target in mavlink.ALIASES.items():
        changed = copy.deepcopy(ir)
        owner = next(item for item in changed['singletons'] if item['name'] == mavlink.OWNER)
        declaration = next(item for item in owner['aliases'] if item['alias'] == alias)
        declaration['name'] = target + '_changed'
        with pytest.raises(ValueError, match=f'unaudited mavlink.{alias}'):
            mavlink.select(changed, 500)


def test_contract_is_bounded_owned_and_target32_capable():
    spec = mavlink.specification(500)
    assert [method['id'] for method in spec['methods']] == [500, 501, 502, 503, 504]
    assert [method['name'] for method in spec['methods']] == [
        'mavlink.init', 'mavlink.register_rx_msgid', 'mavlink.receive_chan', 'mavlink.send_chan',
        'mavlink.block_command',
    ]
    assert 'cleanup-after-worker-join' in spec['methods'][0]['policy']
    assert 'target-system-u32' in spec['methods'][3]['policy']
    assert spec['methods'][2]['result']['message'] == 'bytes298'
    assert 'module-context-owned-command-blocks' in spec['methods'][4]['policy']


def test_runtime_and_gcs_route_without_lua_dependency():
    root = Path(__file__).parents[1]
    runtime = (root / 'Runtime.cpp').read_text()
    cleanup = runtime.index('binding_mavlink_cleanup(&slot);')
    release = runtime.index('Module::release(slot.module);', cleanup)
    assert cleanup < release
    gcs = (root.parent / 'GCS_MAVLink/GCS_Common.cpp').read_text()
    rust_route = gcs.index('AP_Module::binding_mavlink_handle_message(msg, chan);')
    lua_end = gcs.rindex('#endif // AP_SCRIPTING_LUA_ENABLED', 0, rust_route)
    assert lua_end < rust_route
    assert '<AP_Scripting/AP_Scripting.h>' not in (root / 'BindingMAVLink.cpp').read_text()
    assert gcs.count('AP_Module::binding_mavlink_is_handling_command(packet.command)') == 2


def test_block_command_coverage_and_dispatch():
    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    block = coverage[(mavlink.OWNER, 'block_command')]
    assert block['status'] == 'generated'
    assert block['api'] == 'mavlink.block_command'
    assert 'return binding_mavlink_block_command' in bindings.emit(spec)['Bindings_generated.cpp']


def test_emitted_rust_sdk_wire_and_validation(tmp_path):
    generated = mavlink.rust(mavlink.specification(500), '')
    source = tmp_path / 'mavlink.rs'
    source.write_text('''
use core::ffi::c_void;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error { Invalid, NoMemory, Host }
pub type Result<T = ()> = core::result::Result<T, Error>;
fn result(status: i32) -> Result {
    match status { 0 => Ok(()), -1 => Err(Error::Invalid), -2 => Err(Error::NoMemory), _ => Err(Error::Host) }
}
struct HostApi {
    binding_call: Option<unsafe extern "C" fn(*mut c_void, u32, *const u8, u32, *mut u8, u32) -> i32>,
}
pub struct Context<'a> { api: &'a HostApi, raw: *mut c_void }
pub mod bindings {
    pub struct Mavlink<'context, 'api> {
        pub(crate) context: &'context crate::Context<'api>,
    }
}
impl<'api> Context<'api> {
    pub fn mavlink(&self) -> bindings::Mavlink<'_, 'api> {
        bindings::Mavlink { context: self }
    }
}
pub(crate) struct Import(u32);
impl Import { pub(crate) fn slot(&self) -> u32 { self.0 } }
pub(crate) mod imports {
    use super::Import;
    pub(crate) static BINDING_500: Import = Import(500);
    pub(crate) static BINDING_501: Import = Import(501);
    pub(crate) static BINDING_502: Import = Import(502);
    pub(crate) static BINDING_503: Import = Import(503);
    pub(crate) static BINDING_504: Import = Import(504);
}
mod generated {
''' + generated + '''
}

#[cfg(test)]
mod tests {
    use super::{generated::*, Context, Error, HostApi};
    use core::ffi::c_void;

    struct Host { status: i32, id: u32, input: [u8; 267], input_len: usize, output: [u8; 304] }

    unsafe extern "C" fn call(raw: *mut c_void, id: u32, input: *const u8, ni: u32,
                               output: *mut u8, no: u32) -> i32 {
        let host = &mut *raw.cast::<Host>();
        host.id = id; host.input_len = ni as usize;
        if ni != 0 { host.input[..ni as usize].copy_from_slice(core::slice::from_raw_parts(input, ni as usize)); }
        if no != 0 { core::slice::from_raw_parts_mut(output, no as usize).copy_from_slice(&host.output[..no as usize]); }
        host.status
    }

    fn context(host: &mut Host) -> (HostApi, Context<'_>) {
        let api = HostApi { binding_call: Some(call) };
        // Extend only for this test expression; Context never outlives `api` below.
        let api_ref = unsafe { &*(&api as *const HostApi) };
        let context = Context { api: api_ref, raw: (host as *mut Host).cast() };
        (api, context)
    }

    fn host() -> Host { Host { status: 0, id: 0, input: [0; 267], input_len: 0, output: [0; 304] } }

    #[test]
    fn send_encodes_payload_and_full_target() {
        let mut host = host();
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        host.output[..2].copy_from_slice(&[1, 1]);
        assert_eq!(context.mavlink().send_chan(3, 345, &[1, 2, 3], Some(0x12345678)), Ok(Some(true)));
        assert_eq!((host.id, host.input_len), (503, 267));
        assert_eq!(host.input[0], 3);
        assert_eq!(&host.input[1..5], &345u32.to_le_bytes());
        assert_eq!(&host.input[5..7], &3u16.to_le_bytes());
        assert_eq!(host.input[7], 1);
        assert_eq!(&host.input[8..12], &0x12345678u32.to_le_bytes());
        assert_eq!(&host.input[12..15], &[1, 2, 3]);
        assert!(host.input[15..].iter().all(|byte| *byte == 0));
        assert_eq!(context.mavlink().send_chan(0, 1 << 24, &[], None), Err(Error::Invalid));
        assert_eq!(host.id, 503);
    }

    #[test]
    fn receive_copies_complete_message_and_metadata() {
        let mut host = host();
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        host.output[0] = 1;
        for (index, byte) in host.output[1..299].iter_mut().enumerate() { *byte = index as u8; }
        host.output[299] = 4;
        host.output[300..304].copy_from_slice(&0x89abcdefu32.to_le_bytes());
        let message = context.mavlink().receive_chan().unwrap().unwrap();
        assert_eq!(message.bytes()[0], 0);
        assert_eq!(message.bytes()[297], 41);
        assert_eq!((message.channel, message.timestamp_ms), (4, 0x89abcdef));
        host.output = [0; 304];
        assert_eq!(context.mavlink().receive_chan(), Ok(None));
        host.output[12] = 1;
        assert_eq!(context.mavlink().receive_chan(), Err(Error::Host));
    }

    #[test]
    fn init_and_register_are_exact_and_bounded() {
        let mut host = host();
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        assert_eq!(context.mavlink().init(5, 1), Ok(()));
        assert_eq!((host.id, host.input_len, &host.input[..2]), (500, 2, &[5, 1][..]));
        assert_eq!(context.mavlink().init(26, 1), Err(Error::Invalid));
        host.output[0] = 1;
        assert_eq!(context.mavlink().register_rx_msgid(23), Ok(true));
        assert_eq!((host.id, host.input_len, &host.input[..4]), (501, 4, &23u32.to_le_bytes()[..]));
        host.output[0] = 2;
        assert_eq!(context.mavlink().register_rx_msgid(23), Err(Error::Host));
    }

    #[test]
    fn block_command_uses_an_owned_u16_registration() {
        let mut host = host();
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        host.output[0] = 1;
        assert_eq!(context.mavlink().block_command(0x1234), Ok(true));
        assert_eq!((host.id, host.input_len), (504, 2));
        assert_eq!(&host.input[..2], &0x1234u16.to_le_bytes());
        host.output[0] = 2;
        assert_eq!(context.mavlink().block_command(1), Err(Error::Host));
    }
}
''')
    subprocess.run(['rustc', '--edition=2021', '--test', str(source), '-o', str(tmp_path / 'test')], check=True)
    subprocess.run([str(tmp_path / 'test')], check=True)


@pytest.fixture
def host_service(tmp_path):
    (tmp_path / 'AP_HAL/utility').mkdir(parents=True)
    (tmp_path / 'GCS_MAVLink').mkdir()
    (tmp_path / 'AP_HAL/AP_HAL_Boards.h').write_text('#pragma once\n')
    (tmp_path / 'AP_HAL/Semaphores.h').write_text(r'''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL {
struct Semaphore { unsigned depth{}; bool take(uint32_t) { depth++; return true; } void give() { depth--; } };
struct Guard { Semaphore &sem; Guard(Semaphore &s):sem(s){sem.take(0);} ~Guard(){sem.give();} };
}
using HAL_Semaphore = AP_HAL::Semaphore;
#define AP_MODULE_JOIN2(a,b) a##b
#define AP_MODULE_JOIN(a,b) AP_MODULE_JOIN2(a,b)
#define WITH_SEMAPHORE(s) AP_HAL::Guard AP_MODULE_JOIN(_guard_,__LINE__)(s)
''')
    (tmp_path / 'AP_HAL/AP_HAL.h').write_text(r'''#pragma once
#include <AP_HAL/Semaphores.h>
#include <new>
#include <stdint.h>
#define NEW_NOTHROW new(std::nothrow)
#define MIN(a,b) ((a)<(b)?(a):(b))
extern uint32_t test_millis;
namespace AP_HAL { inline uint32_t millis() { return test_millis; } }
''')
    (tmp_path / 'AP_HAL/utility/RingBuffer.h').write_text(r'''#pragma once
#include <deque>
#include <stdint.h>
template <class T> class ObjectBuffer {
public:
    explicit ObjectBuffer(uint32_t size): limit(size) {}
    uint32_t get_size() const { return limit; }
    bool push(const T &value) { if (items.size() >= limit) return false; items.push_back(value); return true; }
    bool pop(T &value) { if (items.empty()) return false; value=items.front(); items.pop_front(); return true; }
private: uint32_t limit; std::deque<T> items;
};
''')
    (tmp_path / 'GCS_MAVLink/GCS_MAVLink.h').write_text(r'''#pragma once
#include <AP_HAL/Semaphores.h>
#include <stdint.h>
#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_SIGNATURE_BLOCK_LEN 13
#define MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM 1
#define MAVLINK_STATUS_FLAG_OUT_MAVLINK1 1
enum mavlink_channel_t : uint8_t { MAVLINK_COMM_0=0, MAVLINK_COMM_1=1, MAVLINK_COMM_2=2, MAVLINK_COMM_3=3 };
struct __attribute__((packed)) mavlink_message_t {
    uint16_t checksum; uint8_t magic,len,incompat_flags,compat_flags,seq; uint32_t sysid; uint8_t compid;
    uint32_t msgid:24; uint64_t payload64[33]; uint8_t ck[2]; uint8_t signature[13]; uint32_t target_sysid;
};
struct mavlink_msg_entry_t { uint32_t msgid; uint8_t crc_extra,min_msg_len,max_msg_len,flags,target_system_ofs; };
struct mavlink_status_t { uint8_t flags; };
const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid);
uint8_t mavlink_msg_target_field(uint32_t target);
mavlink_status_t *mavlink_get_channel_status(mavlink_channel_t chan);
uint16_t comm_get_txspace(mavlink_channel_t chan);
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan);
void _mav_finalize_message_chan_send_target(mavlink_channel_t chan, uint32_t msgid, const char *payload,
                                            uint8_t min_len, uint8_t max_len, uint8_t crc, uint32_t target);
''')
    (tmp_path / 'GCS_MAVLink/GCS.h').write_text(r'''#pragma once
#include <GCS_MAVLink/GCS_MAVLink.h>
struct GCS { uint8_t channels=2; uint8_t num_gcs() const { return channels; } };
GCS &gcs();
struct GCS_MAVLINK { static uint8_t packet_overhead_chan(mavlink_channel_t) { return 12; } };
''')
    harness = tmp_path / 'host.cpp'
    service = Path(__file__).parents[1] / 'BindingMAVLink.cpp'
    harness.write_text(f'''#include <string.h>
#include "{service}"
uint32_t test_millis = 0;
static GCS test_gcs;
static HAL_Semaphore channel_locks[4];
static mavlink_status_t statuses[4];
static uint16_t txspace[4] = {{1024,1024,1024,1024}};
static mavlink_msg_entry_t entries[] = {{{{23,1,23,23,1,4}},{{180,2,45,47,0,0}},{{345,3,21,21,1,2}}}};
static unsigned sends; static uint32_t sent_target; static uint8_t sent_payload[255];
GCS &gcs() {{ return test_gcs; }}
const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t id) {{
    for (auto &entry:entries) {{
        if (entry.msgid==id) {{ return &entry; }}
    }}
    return nullptr;
}}
uint8_t mavlink_msg_target_field(uint32_t target) {{ return target > 255 ? 0 : uint8_t(target); }}
mavlink_status_t *mavlink_get_channel_status(mavlink_channel_t chan) {{ return &statuses[uint8_t(chan)]; }}
uint16_t comm_get_txspace(mavlink_channel_t chan) {{ return txspace[uint8_t(chan)]; }}
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan) {{ return channel_locks[uint8_t(chan)]; }}
void _mav_finalize_message_chan_send_target(mavlink_channel_t, uint32_t, const char *payload,
                                            uint8_t, uint8_t max_len, uint8_t, uint32_t target) {{
    sends++; sent_target=target; memset(sent_payload,0,sizeof(sent_payload)); memcpy(sent_payload,payload,max_len);
}}
extern "C" int init(void *ctx,uint8_t q,uint8_t n) {{
    uint8_t in[2]{{q,n}}; return AP_Module::binding_mavlink_init(ctx,in,2,nullptr,0);
}}
extern "C" int reg(void *ctx,uint32_t id,uint8_t *out) {{
    return AP_Module::binding_mavlink_register_rx_msgid(ctx,(uint8_t*)&id,4,out,1);
}}
extern "C" int receive(void *ctx,uint8_t *out) {{ return AP_Module::binding_mavlink_receive_chan(ctx,nullptr,0,out,304); }}
extern "C" void inject(uint32_t id,uint8_t chan,uint8_t seed,uint32_t now) {{
    mavlink_message_t msg; uint8_t *raw=(uint8_t*)&msg; for (unsigned i=0;i<sizeof(msg);i++) raw[i]=uint8_t(seed+i);
    msg.msgid=id; test_millis=now; AP_Module::binding_mavlink_handle_message(msg,mavlink_channel_t(chan));
}}
extern "C" void cleanup(void *ctx) {{ AP_Module::binding_mavlink_cleanup(ctx); }}
extern "C" unsigned owner_count() {{
 unsigned count=0;
 for(auto *entry=AP_Module::registry;entry;entry=entry->next)count++;
 return count;
}}
extern "C" int send(void *ctx,const uint8_t *in,uint8_t *out) {{
    return AP_Module::binding_mavlink_send_chan(ctx,in,267,out,2);
}}
extern "C" int block(void *ctx,uint16_t id,uint8_t *out) {{
    return AP_Module::binding_mavlink_block_command(ctx,(const uint8_t*)&id,2,out,1);
}}
extern "C" bool handling(uint16_t id) {{ return AP_Module::binding_mavlink_is_handling_command(id); }}
extern "C" unsigned block_count() {{
    unsigned count=0;
    for (const auto *item=AP_Module::command_blocks; item != nullptr; item=item->next) count++;
    return count;
}}
extern "C" void configure(uint8_t channels,uint8_t chan,uint8_t mav1,uint16_t space) {{
    test_gcs.channels=channels; statuses[chan].flags=mav1; txspace[chan]=space;
}}
extern "C" unsigned send_count() {{ return sends; }}
extern "C" uint32_t target() {{ return sent_target; }}
extern "C" uint8_t payload(unsigned index) {{ return sent_payload[index]; }}
''')
    library = tmp_path / 'host.so'
    subprocess.run([
        os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
        '-DAP_MODULE_ENABLED=1', '-DHAL_GCS_ENABLED=1', '-I' + str(tmp_path),
        '-I' + str(Path(__file__).parents[1]), str(harness), '-o', str(library),
    ], check=True)
    lib = ctypes.CDLL(str(library))
    lib.init.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8]
    lib.reg.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p]
    lib.receive.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.inject.argtypes = [ctypes.c_uint32, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint32]
    lib.cleanup.argtypes = [ctypes.c_void_p]
    lib.send.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    lib.block.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_void_p]
    lib.handling.argtypes = [ctypes.c_uint16]
    lib.handling.restype = ctypes.c_bool
    lib.block_count.restype = ctypes.c_uint
    lib.configure.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16]
    lib.target.restype = ctypes.c_uint32
    lib.payload.argtypes = [ctypes.c_uint]
    lib.payload.restype = ctypes.c_uint8
    return lib


def test_host_service_isolates_queues_and_cleans_after_join(host_service):
    lib = host_service
    first, second = ctypes.c_int(1), ctypes.c_int(2)
    first_ctx, second_ctx = ctypes.byref(first), ctypes.byref(second)
    assert lib.init(first_ctx, 1, 1) == 0
    assert lib.init(second_ctx, 2, 1) == 0
    added = ctypes.create_string_buffer(1)
    assert lib.reg(first_ctx, 23, added) == 0 and added.raw == b'\x01'
    assert lib.reg(first_ctx, 23, added) == 0 and added.raw == b'\x00'
    assert lib.reg(second_ctx, 180, added) == 0 and added.raw == b'\x01'
    assert lib.reg(first_ctx, 24, added) == -2

    lib.inject(23, 1, 10, 1000)
    lib.inject(23, 2, 20, 2000)  # first queue is full; newest is dropped
    lib.inject(180, 3, 30, 3000)
    output = ctypes.create_string_buffer(mavlink.RECEIVE_WIRE_SIZE)
    assert lib.receive(first_ctx, output) == 0
    assert output.raw[0] == 1 and output.raw[299] == 1
    assert output.raw[300:304] == (1000).to_bytes(4, 'little')
    assert output.raw[1:3] == bytes([10, 11])
    assert lib.receive(first_ctx, output) == 0 and output.raw == bytes(mavlink.RECEIVE_WIRE_SIZE)
    assert lib.receive(second_ctx, output) == 0
    assert output.raw[0] == 1 and output.raw[299] == 3 and output.raw[300:304] == (3000).to_bytes(4, 'little')

    lib.cleanup(first_ctx)
    assert lib.receive(first_ctx, output) == -1
    assert lib.receive(second_ctx, output) == 0
    lib.cleanup(second_ctx)


def test_host_command_blocks_are_owner_scoped_and_cleaned(host_service):
    lib = host_service
    first, second = ctypes.c_int(11), ctypes.c_int(12)
    first_ctx, second_ctx = ctypes.byref(first), ctypes.byref(second)
    output = ctypes.create_string_buffer(1)
    assert lib.block(first_ctx, 31000, output) == 0 and output.raw == b'\x01'
    assert lib.block(first_ctx, 31000, output) == 0 and lib.block_count() == 1
    assert lib.block(second_ctx, 31000, output) == 0 and lib.block_count() == 2
    assert lib.handling(31000)
    lib.cleanup(first_ctx)
    assert lib.handling(31000) and lib.block_count() == 1
    lib.cleanup(second_ctx)
    assert not lib.handling(31000) and lib.block_count() == 0


def test_more_than_eight_mavlink_owners_and_cleanup(host_service):
    lib = host_service
    contexts = [ctypes.c_int(i) for i in range(32)]
    added = ctypes.create_string_buffer(1)
    output = ctypes.create_string_buffer(mavlink.RECEIVE_WIRE_SIZE)
    for _ in range(4):
        for context in contexts:
            assert lib.init(ctypes.byref(context), 1, 1) == 0
            assert lib.reg(ctypes.byref(context), 23, added) == 0
        assert lib.owner_count() == 32
        lib.inject(23, 1, 10, 1000)
        for context in contexts:
            assert lib.receive(ctypes.byref(context), output) == 0 and output.raw[0] == 1
            lib.cleanup(ctypes.byref(context))
            lib.cleanup(ctypes.byref(context))
        assert lib.owner_count() == 0


def send_wire(channel, msgid, payload=b'', target=None):
    wire = bytearray(mavlink.SEND_WIRE_SIZE)
    wire[0] = channel
    wire[1:5] = msgid.to_bytes(4, 'little')
    wire[5:7] = len(payload).to_bytes(2, 'little')
    if target is not None:
        wire[7] = 1
        wire[8:12] = target.to_bytes(4, 'little')
    wire[12:12 + len(payload)] = payload
    return bytes(wire)


def test_host_send_preserves_full_target_and_lua_results(host_service):
    lib = host_service
    context = ctypes.byref(ctypes.c_int(3))
    output = ctypes.create_string_buffer(2)
    packet = send_wire(0, 345, bytes(range(21)), 0x12345678)
    assert lib.send(context, packet, output) == 0
    assert output.raw == b'\x01\x01'
    assert lib.send_count() == 1 and lib.target() == 0x12345678
    assert lib.payload(2) == 0  # wide target sentinel replaces the payload target byte

    lib.configure(2, 0, 1, 1024)
    assert lib.send(context, packet, output) == 0 and output.raw == b'\x01\x00'
    assert lib.send_count() == 1
    lib.configure(1, 0, 0, 1024)
    assert lib.send(context, send_wire(1, 180), output) == 0 and output.raw == b'\x00\x00'
    assert lib.send(context, send_wire(0, 180, target=2), output) == -1

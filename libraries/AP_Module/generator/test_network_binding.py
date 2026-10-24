# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate module-owned bounded socket contracts, SDK wire and cleanup."""

import copy
import ctypes
import os
import struct
import subprocess

from pathlib import Path

import bindings
import network_binding as network
import pytest


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert network.select(ir, 800) == network.specification(800)
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['ap_objects'] if item['name'] == network.OWNER)
    next(item for item in owner['methods'] if item['name'] == 'pollin')['arguments'][0]['kind'] = 'uint16_t'
    with pytest.raises(ValueError, match='SocketAPM methods'):
        network.select(changed, 800)


def test_contract_is_owned_and_bounded():
    spec = network.specification(800)
    assert len(spec['methods']) == 17
    assert [method['id'] for method in spec['methods']] == list(range(800, 817))
    assert [method['name'] for method in spec['methods'][-4:]] == [
        'network.socket_recv_from',
        'network.socket_is_pending',
        'network.socket_reuse_address',
        'network.socket_send_to',
    ]
    recv = spec['methods'][9]
    assert recv['id'] == 809 and recv['name'] == 'network.socket_recv'
    assert recv['result'] == dict(presence='bool-u8', length='u16-le',
                                  data='bytes-max1024-zero-pad1024')
    policy = spec['methods'][0]['policy']
    assert 'cleanup-after-worker-join' in policy
    assert 'descriptor-always-nonblocking' in policy
    assert 'generation-seeds-persist-across-context-restart' in policy
    assert 'transfer-max1024' in policy


def test_runtime_cleanup_is_after_join_and_before_release():
    runtime = (Path(__file__).parents[1] / 'Runtime.cpp').read_text()
    cleanup = runtime.index('binding_network_cleanup(&slot);')
    release = runtime.index('Module::release(slot.module);', cleanup)
    assert cleanup < release


def test_remaining_network_coverage_and_dispatch_are_wired():
    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    assert coverage[(network.OWNER, 'reuseaddress')]['api'] == 'network.socket_reuse_address'
    assert coverage[(network.OWNER, 'sendto')]['api'] == 'network.socket_send_to'
    assert coverage[(network.NETWORK_OWNER, 'address_to_str')]['api'] == 'Ipv4Address'
    source = bindings.emit(spec)['Bindings_generated.cpp']
    assert 'return binding_network_reuse_address' in source
    assert 'return binding_network_send_to' in source


def test_host_compiles_with_networking_disabled(tmp_path):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': r'''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t){return true;} void give(){} }; }
using HAL_Semaphore=AP_HAL::Semaphore;
''',
        'AP_HAL/AP_HAL.h': '#pragma once\n#include <AP_HAL/Semaphores.h>\n',
        'AP_Networking/AP_Networking_Config.h': '#pragma once\n#define AP_NETWORKING_ENABLED 0\n',
        'AP_HAL/utility/Socket.h': '#pragma once\n',
        'AP_Filesystem/AP_Filesystem.h': '#pragma once\n',
        'AP_Networking/AP_Networking.h': '#pragma once\n',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    service = Path(__file__).parents[1] / 'BindingNetwork.cpp'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-DAP_MODULE_ENABLED=1', '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
                    '-c', str(service), '-o', str(tmp_path / 'network.o')], check=True)


@pytest.fixture
def host_service(tmp_path):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': r'''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t){return true;} void give(){} };
struct Guard { Semaphore&s; Guard(Semaphore&v):s(v){s.take(0);} ~Guard(){s.give();} }; }
using HAL_Semaphore=AP_HAL::Semaphore;
#define JOIN2(a,b) a##b
#define JOIN(a,b) JOIN2(a,b)
#define WITH_SEMAPHORE(s) AP_HAL::Guard JOIN(g_,__LINE__)(s)
''',
        'AP_HAL/AP_HAL.h': '''#pragma once
#include <AP_HAL/Semaphores.h>
#include <new>
#define NEW_NOTHROW new(std::nothrow)
''',
        'AP_Networking/AP_Networking_Config.h': '''#pragma once
#define AP_NETWORKING_ENABLED 1
''',
        'AP_HAL/utility/Socket.h': r'''#pragma once
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
class SocketAPM {
public:
    explicit SocketAPM(bool); ~SocketAPM(); bool set_blocking(bool); bool bind(const char*,uint16_t);
    bool connect(const char*,uint16_t); bool is_pending() const { return pending; }
    bool listen(uint16_t) const; bool is_connected() const; bool pollin(uint32_t); bool pollout(uint32_t);
    SocketAPM *accept(uint32_t); ssize_t recv(void*,size_t,uint32_t); ssize_t send(const void*,size_t) const;
    ssize_t sendto(const void*,size_t,uint32_t,uint16_t); bool reuseaddress() const;
    bool last_recv_address(uint32_t&,uint16_t&) const;
    SocketAPM *duplicate();
private: bool valid=true; bool connected=false; bool pending=false;
};
''',
        'AP_Filesystem/AP_Filesystem.h': r'''#pragma once
#include <stdint.h>
class AP_Filesystem { public: int open(const char*,int); int close(int); };
namespace AP { AP_Filesystem &FS(); }
''',
        'AP_Networking/AP_Networking.h': r'''#pragma once
class SocketAPM;
class AP_Networking { public: bool sendfile(SocketAPM*,int); };
namespace AP { AP_Networking &network(); }
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    service = Path(__file__).parents[1] / 'BindingNetwork.cpp'
    harness = tmp_path / 'host.cpp'
    harness.write_text(f'''#include <string.h>
#include "{service}"
static unsigned live, accepted_ready, sendfile_count, file_live;
static bool input_ready, output_ready=true, peer_available;
static uint32_t peer_address; static uint16_t peer_port;
static unsigned sent_len; static uint8_t sent_data[1024];
static uint32_t sent_address; static uint16_t sent_port; static bool reused;
SocketAPM::SocketAPM(bool){{live++;}}
SocketAPM::~SocketAPM(){{live--;}}
bool SocketAPM::set_blocking(bool value){{return !value;}}
bool SocketAPM::bind(const char *address,uint16_t port){{
 return !strcmp(address,"0.0.0.0")&&port==8081;
}}
bool SocketAPM::connect(const char *address,uint16_t port){{
 if(strcmp(address,"caster")||port!=2101) return false;
 pending=true; return false;
}}
bool SocketAPM::listen(uint16_t backlog)const{{return backlog>0;}}
bool SocketAPM::is_connected()const{{return connected;}}
bool SocketAPM::pollin(uint32_t timeout){{return input_ready||accepted_ready||(timeout==20&&input_ready);}}
bool SocketAPM::pollout(uint32_t timeout){{
 if(pending&&output_ready&&timeout==20){{pending=false;connected=true;}}
 return output_ready;
}}
SocketAPM *SocketAPM::accept(uint32_t){{
 if(!accepted_ready){{errno=EWOULDBLOCK;return nullptr;}}
 accepted_ready--; auto*s=new SocketAPM(false); s->connected=true; return s;
}}
ssize_t SocketAPM::recv(void *out,size_t size,uint32_t){{
 if(!input_ready){{errno=EWOULDBLOCK;return -1;}} const char data[]="request";
 size_t n=size<sizeof(data)-1?size:sizeof(data)-1; memcpy(out,data,n); input_ready=false; return n;
}}
bool SocketAPM::last_recv_address(uint32_t &address,uint16_t &port)const{{
 if(!peer_available){{return false;}}
 address=peer_address;port=peer_port;return true;
}}
ssize_t SocketAPM::send(const void *data,size_t size)const{{
 if(!output_ready){{errno=EWOULDBLOCK;return -1;}} sent_len=size; memcpy(sent_data,data,size); return size;
}}
ssize_t SocketAPM::sendto(const void *data,size_t size,uint32_t address,uint16_t port){{
 if(!output_ready){{errno=EWOULDBLOCK;return -1;}}
 sent_len=size;sent_address=address;sent_port=port;memcpy(sent_data,data,size);return size;
}}
bool SocketAPM::reuseaddress()const{{reused=true;return true;}}
SocketAPM *SocketAPM::duplicate(){{valid=false;return new SocketAPM(false);}}
static AP_Filesystem fs; AP_Filesystem &AP::FS(){{return fs;}}
int AP_Filesystem::open(const char *path,int flags){{
 if(flags!=O_RDONLY){{__builtin_trap();}}
 if(strcmp(path,"file"))return -1;
 file_live++;return 9;
}}
int AP_Filesystem::close(int fd){{if(fd!=9)__builtin_trap();file_live--;return 0;}}
static AP_Networking net; AP_Networking &AP::network(){{return net;}}
bool AP_Networking::sendfile(SocketAPM *sock,int fd){{
 if(fd!=9){{__builtin_trap();}}
 auto*owned=sock->duplicate(); delete owned; file_live--; sendfile_count++; return true;
}}
extern "C" int open_call(void*c,uint8_t*d,uint8_t*out){{return AP_Module::binding_network_open(c,d,1,out,4);}}
extern "C" int bind_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_bind(c,(const uint8_t*)in,262,out,1);}}
extern "C" int connect_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_connect(c,(const uint8_t*)in,262,out,1);}}
extern "C" int blocking_call(void*c,const void*in){{
 return AP_Module::binding_network_set_blocking(c,(const uint8_t*)in,5,nullptr,0);}}
extern "C" int accept_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_accept(c,(const uint8_t*)in,4,out,4);}}
extern "C" int recv_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_recv(c,(const uint8_t*)in,6,out,1027);}}
extern "C" int recv_from_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_recv_from(c,(const uint8_t*)in,6,out,1034);}}
extern "C" int pending_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_is_pending(c,(const uint8_t*)in,4,out,1);}}
extern "C" int reuse_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_reuse_address(c,(const uint8_t*)in,4,out,1);}}
extern "C" int send_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_send(c,(const uint8_t*)in,1030,out,3);}}
extern "C" int sendto_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_send_to(c,(const uint8_t*)in,1036,out,3);}}
extern "C" int sendfile_call(void*c,const void*in,uint8_t*out){{
 return AP_Module::binding_network_sendfile(c,(const uint8_t*)in,260,out,1);}}
extern "C" int close_call(void*c,const void*in){{return AP_Module::binding_network_close(c,(const uint8_t*)in,4,nullptr,0);}}
extern "C" void cleanup(void*c){{AP_Module::binding_network_cleanup(c);}}
extern "C" void ready(unsigned accepts,bool input,bool output){{accepted_ready=accepts;input_ready=input;output_ready=output;}}
extern "C" void peer(bool available,uint32_t address,uint16_t port){{
 peer_available=available;peer_address=address;peer_port=port;
}}
extern "C" unsigned live_count(){{return live;}}
extern "C" unsigned owner_count(){{
 unsigned count=0;
 for(auto *entry=AP_Module::registry;entry;entry=entry->next)count++;
 return count;
}}
extern "C" unsigned files(){{return file_live;}}
extern "C" unsigned sent(){{return sent_len;}}
extern "C" uint32_t destination_address(){{return sent_address;}}
extern "C" uint16_t destination_port(){{return sent_port;}}
extern "C" bool was_reused(){{return reused;}}
extern "C" int exhausted_tokens(){{
 AP_Module::next_generation=AP_Module::TOKEN_GENERATION_MASK;
 AP_Module::ModuleSockets sockets;
 uint32_t first=0,second=0;
 sockets.add(new SocketAPM(false),first);
 auto *slot=sockets.resolve(first);
 sockets.release(*slot);
 return sockets.add(new SocketAPM(false),second);
}}
''')
    library = tmp_path / 'host.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-fPIC', '-shared', '-DAP_MODULE_ENABLED=1',
                    '-DAP_FILESYSTEM_FILE_READING_ENABLED=1', '-I' + str(tmp_path),
                    '-I' + str(bindings.ROOT), str(harness), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.open_call.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    for name in ('bind_call', 'connect_call', 'accept_call', 'recv_call', 'recv_from_call',
                 'pending_call', 'reuse_call', 'send_call', 'sendto_call', 'sendfile_call'):
        getattr(lib, name).argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
    lib.blocking_call.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.close_call.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.cleanup.argtypes = [ctypes.c_void_p]
    lib.ready.argtypes = [ctypes.c_uint, ctypes.c_bool, ctypes.c_bool]
    lib.peer.argtypes = [ctypes.c_bool, ctypes.c_uint32, ctypes.c_uint16]
    lib.destination_address.restype = ctypes.c_uint32
    lib.destination_port.restype = ctypes.c_uint16
    lib.was_reused.restype = ctypes.c_bool
    return lib


def open_socket(lib, context):
    output = ctypes.create_string_buffer(4)
    datagram = ctypes.create_string_buffer(b'\0')
    assert lib.open_call(context, datagram, output) == 0
    return output.raw


def test_host_ownership_io_accept_sendfile_and_cleanup(host_service):
    lib = host_service
    first = ctypes.c_int(1)
    second = ctypes.c_int(2)
    token = open_socket(lib, ctypes.byref(first))
    assert lib.live_count() == 1
    packet = token + b'0.0.0.0'.ljust(256, b'\0') + struct.pack('<H', 8081)
    one = ctypes.create_string_buffer(1)
    assert lib.bind_call(ctypes.byref(first), packet, one) == 0 and one.raw == b'\1'
    assert lib.bind_call(ctypes.byref(second), packet, one) == -1
    assert lib.reuse_call(ctypes.byref(first), token, one) == 0 and one.raw == b'\1'
    assert lib.was_reused()

    lib.ready(1, True, True)
    child = ctypes.create_string_buffer(4)
    assert lib.accept_call(ctypes.byref(first), token, child) == 0
    assert child.raw != bytes(4) and lib.live_count() == 2
    received = ctypes.create_string_buffer(1027)
    assert lib.recv_call(ctypes.byref(first), child.raw + struct.pack('<H', 16), received) == 0
    assert received.raw[:10] == b'\1\7\0request'
    assert received.raw[10:] == bytes(1017)

    lib.ready(0, True, True)
    lib.peer(True, 0x7F000001, 14550)
    received_from = ctypes.create_string_buffer(1034)
    assert lib.recv_from_call(ctypes.byref(first), child.raw + struct.pack('<H', 16), received_from) == 0
    assert received_from.raw[:10] == (b'\1\7\0\1' + struct.pack('<IH', 0x7F000001, 14550))
    assert received_from.raw[10:17] == b'request'
    assert received_from.raw[17:] == bytes(1017)

    lib.ready(0, True, True)
    lib.peer(False, 0xFFFFFFFF, 65535)
    received_from = ctypes.create_string_buffer(1034)
    assert lib.recv_from_call(ctypes.byref(first), child.raw + struct.pack('<H', 16), received_from) == 0
    assert received_from.raw[:10] == b'\1\7\0' + bytes(7)
    assert received_from.raw[10:17] == b'request'

    lib.ready(0, False, True)
    received_from = ctypes.create_string_buffer(1034)
    assert lib.recv_from_call(ctypes.byref(first), child.raw + struct.pack('<H', 16), received_from) == 0
    assert received_from.raw == bytes(1034)
    send_packet = child.raw + struct.pack('<H', 5) + b'reply' + bytes(1019)
    sent = ctypes.create_string_buffer(3)
    assert lib.send_call(ctypes.byref(first), send_packet, sent) == 0
    assert sent.raw == b'\1\5\0' and lib.sent() == 5
    sendto_packet = (child.raw + struct.pack('<HIH', 5, 0x7F000001, 14550) +
                     b'reply' + bytes(1019))
    assert lib.sendto_call(ctypes.byref(first), sendto_packet, sent) == 0
    assert sent.raw == b'\1\5\0' and lib.sent() == 5
    assert lib.destination_address() == 0x7F000001 and lib.destination_port() == 14550

    sendfile_packet = child.raw + b'file'.ljust(256, b'\0')
    assert lib.sendfile_call(ctypes.byref(first), sendfile_packet, one) == 0 and one.raw == b'\1'
    assert lib.files() == 0 and lib.close_call(ctypes.byref(first), child.raw) == -1
    lib.cleanup(ctypes.byref(first))
    assert lib.live_count() == 0


def test_host_connect_is_bounded_and_restart_tokens_do_not_alias(host_service):
    lib = host_service
    context = ctypes.c_int(3)
    token = open_socket(lib, ctypes.byref(context))
    packet = token + b'caster'.ljust(256, b'\0') + struct.pack('<H', 2101)
    one = ctypes.create_string_buffer(1)

    # Nonblocking connect starts but does not wait for completion.
    lib.ready(0, False, True)
    assert lib.connect_call(ctypes.byref(context), packet, one) == 0 and one.raw == b'\0'
    assert lib.pending_call(ctypes.byref(context), token, one) == 0 and one.raw == b'\1'
    other = ctypes.c_int(4)
    assert lib.pending_call(ctypes.byref(other), token, one) == -1
    lib.cleanup(ctypes.byref(context))

    # Generations persist independently of freed owner/context addresses.
    restarted = open_socket(lib, ctypes.byref(context))
    assert restarted != token
    assert lib.connect_call(ctypes.byref(context), packet, one) == -1
    assert lib.pending_call(ctypes.byref(context), token, one) == -1
    blocking = restarted + b'\1'
    assert lib.blocking_call(ctypes.byref(context), blocking) == 0
    packet = restarted + b'caster'.ljust(256, b'\0') + struct.pack('<H', 2101)
    assert lib.connect_call(ctypes.byref(context), packet, one) == 0 and one.raw == b'\1'
    assert lib.pending_call(ctypes.byref(context), restarted, one) == 0 and one.raw == b'\0'
    lib.cleanup(ctypes.byref(context))

    # Exhaustion fails closed rather than making a retired handle valid again.
    assert lib.exhausted_tokens() == -2
    assert lib.live_count() == 0


def test_more_than_eight_socket_owners_and_reused_addresses(host_service):
    lib = host_service
    contexts = [ctypes.c_int(i) for i in range(32)]
    previous = set()
    for _ in range(4):
        tokens = [open_socket(lib, ctypes.byref(context)) for context in contexts]
        assert not previous.intersection(tokens)
        assert lib.owner_count() == 32 and lib.live_count() == 32
        for context, token in zip(contexts, tokens):
            assert lib.close_call(ctypes.byref(context), token) == 0
            lib.cleanup(ctypes.byref(context))
            lib.cleanup(ctypes.byref(context))  # idempotent after freeing the registry node
        assert lib.owner_count() == 0 and lib.live_count() == 0
        previous.update(tokens)


def test_emitted_sdk_compiles(tmp_path):
    generated = network.rust(network.specification(800), '')
    source = tmp_path / 'sdk.rs'
    imports = '\n'.join(f'pub(crate) static BINDING_{index}: Import = Import({index});'
                        for index in range(800, 817))
    source.write_text('''
use core::ffi::c_void;
#[derive(Clone,Copy,Debug,Eq,PartialEq)] pub enum Error { Invalid, Host }
pub type Result<T=()> = core::result::Result<T,Error>;
fn result(s:i32)->Result { if s==0 {Ok(())} else {Err(Error::Host)} }
struct HostApi { binding_call: Option<unsafe extern "C" fn(*mut c_void,u32,*const u8,u32,*mut u8,u32)->i32> }
pub struct Context<'a>{api:&'a HostApi,raw:*mut c_void}
pub mod bindings {
    pub struct Network<'context, 'api> {
        pub(crate) context: &'context crate::Context<'api>,
    }
}
impl<'api> Context<'api> {
    pub fn network(&self) -> bindings::Network<'_, 'api> {
        bindings::Network { context: self }
    }
}
pub(crate) struct Import(u32); impl Import {pub(crate) fn slot(&self)->u32{self.0}}
pub(crate) mod imports {use super::Import;
''' + imports + '''
}
mod generated {
''' + generated + r'''
}
fn main() {
    let address = generated::Ipv4Address(0x7f000001);
    assert_eq!(address.octets(), [127, 0, 0, 1]);
    assert_eq!(address.to_string(), "127.0.0.1");
    let peer = generated::Ipv4Endpoint { address: 0x7f000001, port: 14550 };
    assert_eq!(peer.address_octets(), [127, 0, 0, 1]);
    assert_eq!(peer.to_string(), "127.0.0.1:14550");
}
''')
    subprocess.run(['rustc', '--edition=2021', str(source), '-o', str(tmp_path / 'sdk')], check=True)

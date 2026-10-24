# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate compact applet/driver host adapters and emitted Rust APIs."""

import copy
import ctypes
import os
import struct
import subprocess

import applet_binding as applet
import bindings
import pytest


def _owner(ir, name):
    return next(item for item in ir['singletons'] if item['name'] == name)


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert applet.select(ir, 700) == applet.specification(700)
    mutations = [
        (applet.GPS_OWNER, 'time_epoch_usec', 'return', None, 'kind', 'uint32_t'),
        (applet.CAMERA_OWNER, 'set_stream_information', 'arguments', 1, 'kind', 'uint32_t'),
        (applet.NETWORKING_OWNER, 'add_route', 'arguments', 3, 'kind', 'uint16_t'),
        (applet.SCRIPTING_OWNER, 'restart_all', 'return', None, 'kind', 'boolean'),
    ]
    for owner_name, method_name, member, index, field, value in mutations:
        changed = copy.deepcopy(ir)
        method = next(item for item in _owner(changed, owner_name)['methods'] if item['name'] == method_name)
        target = method[member] if index is None else method[member][index]
        target[field] = value
        with pytest.raises(ValueError, match='unaudited'):
            applet.select(changed, 700)

    changed = copy.deepcopy(ir)
    gps = _owner(changed, applet.GPS_OWNER)
    next(alias for alias in gps['aliases'] if alias['alias'] == 'inject_data')['manual'] = False
    with pytest.raises(ValueError, match='AP_GPS.inject_data'):
        applet.select(changed, 700)

    changed = copy.deepcopy(ir)
    value = next(item for item in changed['userdata'] if item['name'] == applet.CAMERA_VALUE)
    next(field for field in value['fields'] if field['name'] == 'uri')['array_len'] = '159'
    with pytest.raises(ValueError, match='video-stream field uri'):
        applet.select(changed, 700)


def test_contract_is_bounded_owned_and_explicit_about_effects():
    spec = applet.specification(700)
    assert [method['id'] for method in spec['methods']] == list(range(700, 705))
    assert spec['gps_inject_max'] == 1024
    assert spec['video_stream_name_size'] == 32
    assert spec['video_stream_uri_size'] == 160
    policies = {method['name']: method['policy'] for method in spec['methods']}
    assert 'inject-nonempty-max1024' in policies['gps.inject_data']
    assert 'owned-video-stream-value' in policies['camera.set_stream_information']
    assert 'camera-device-id-host-owned' in policies['camera.set_stream_information']
    assert 'persistent-route-shared-state' in policies['networking.add_route']
    assert 'rust-self-not-restarted' in policies['scripting.restart_all']


def _packet(name=b'main', uri=b'rtsp://camera/live', framerate=29.5):
    prefix = struct.pack('<BfIHHHHHBBB', 3, framerate, 8_000_000, 0x1234, 1920, 1080, 270, 95, 2, 4, 1)
    return prefix + name.ljust(32, b'\0') + uri.ljust(160, b'\0') + bytes([2])


@pytest.fixture(params=[True, False], ids=['enabled', 'disabled'])
def host(tmp_path, request):
    semaphore = tmp_path / 'AP_HAL/Semaphores.h'
    semaphore.parent.mkdir()
    semaphore.write_text(r'''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL {
struct Semaphore {
    bool available = true; unsigned depth = 0;
    bool take(uint32_t timeout) {
        if (timeout != 0) { __builtin_trap(); }
        depth++; return true;
    }
    void give() { if (depth != 1) { __builtin_trap(); } depth--; }
};
}
''')
    source = tmp_path / 'applet.cpp'
    library = tmp_path / 'applet.so'
    lines = ['#include "BindingSupport.h"', '#include "HostApi.h"', r'''
#include <stdint.h>
#include <string.h>

typedef struct __mavlink_video_stream_information_t {
    float framerate; uint32_t bitrate; uint16_t flags; uint16_t resolution_h; uint16_t resolution_v;
    uint16_t rotation; uint16_t hfov; uint8_t stream_id; uint8_t count; uint8_t type;
    char name[32]; char uri[160]; uint8_t encoding; uint8_t camera_device_id;
} mavlink_video_stream_information_t;

static bool gps_present=true, camera_present=true, networking_present=true, scripting_present=true;
static uint8_t gps_count=2; static uint64_t epoch=UINT64_C(0xfedcba9876543210);
static uint8_t injected[1024]; static uint16_t injected_length; static unsigned inject_calls;
class AP_GPS {
public:
    static AP_GPS *get_singleton() { static AP_GPS value; return gps_present ? &value : nullptr; }
    uint8_t num_sensors() const { return gps_count; }
    uint64_t time_epoch_usec(uint8_t instance) const { return epoch + instance; }
    void inject_data(const uint8_t *data, uint16_t length) {
        memcpy(injected,data,length); injected_length=length; inject_calls++;
    }
};

static AP_HAL::Semaphore camera_sem; static unsigned camera_calls; static uint8_t camera_instance;
static mavlink_video_stream_information_t camera_information;
class AP_Camera {
public:
    static AP_Camera *get_singleton() { static AP_Camera value; return camera_present ? &value : nullptr; }
    AP_HAL::Semaphore &get_semaphore() { return camera_sem; }
    void set_stream_information(uint8_t instance, mavlink_video_stream_information_t information) {
        if (camera_sem.depth != 1) { __builtin_trap(); }
        camera_instance=instance; camera_information=information; camera_calls++;
    }
};

static unsigned route_calls; static uint8_t route_backend,route_iface,route_mask; static uint32_t route_dest;
static bool route_result=true;
class AP_Networking {
public:
    static AP_Networking *get_singleton() { static AP_Networking value; return networking_present ? &value : nullptr; }
    bool add_route(uint8_t backend,uint8_t iface,uint32_t dest,uint8_t mask) {
        route_calls++; route_backend=backend; route_iface=iface; route_dest=dest; route_mask=mask; return route_result;
    }
};

static unsigned restart_calls;
class AP_Scripting {
public:
    static AP_Scripting *get_singleton() { static AP_Scripting value; return scripting_present ? &value : nullptr; }
    void restart_all() { restart_calls++; }
};

namespace AP_Module {
''']
    lines += applet.cpp(applet.specification(700))
    lines += [r'''}
extern "C" int epoch_call(const void*in,uint32_t ni,void*out,uint32_t no) {
    return AP_Module::binding_applet_gps_time_epoch_usec((const uint8_t*)in,ni,(uint8_t*)out,no);
}
extern "C" int inject_call(const void*in,uint32_t ni,void*out,uint32_t no) {
    return AP_Module::binding_applet_gps_inject_data((const uint8_t*)in,ni,(uint8_t*)out,no);
}
extern "C" int camera_call(const void*in,uint32_t ni,void*out,uint32_t no) {
    return AP_Module::binding_applet_camera_set_stream_information((const uint8_t*)in,ni,(uint8_t*)out,no);
}
extern "C" int route_call(const void*in,uint32_t ni,void*out,uint32_t no) {
    return AP_Module::binding_applet_networking_add_route((const uint8_t*)in,ni,(uint8_t*)out,no);
}
extern "C" int restart_call(const void*in,uint32_t ni,void*out,uint32_t no) {
    return AP_Module::binding_applet_scripting_restart_all((const uint8_t*)in,ni,(uint8_t*)out,no);
}
extern "C" void configure(bool gps,bool camera,bool network,bool scripting,bool lock,bool route) {
    gps_present=gps;camera_present=camera;networking_present=network;scripting_present=scripting;
    camera_sem.available=lock;route_result=route;
}
extern "C" unsigned injected_count() { return inject_calls; }
extern "C" unsigned injected_size() { return injected_length; }
extern "C" uint8_t injected_byte(unsigned i) { return injected[i]; }
extern "C" unsigned camera_count() { return camera_calls; }
extern "C" unsigned route_count() { return route_calls; }
extern "C" unsigned restart_count() { return restart_calls; }
extern "C" uint8_t camera_device_id() { return camera_information.camera_device_id; }
extern "C" void read_camera(uint8_t *out) {
    out[0]=camera_instance; AP_Module::BindingWire::write_float(out+1,camera_information.framerate);
    AP_Module::BindingWire::write(out+5,camera_information.bitrate,4);
    AP_Module::BindingWire::write(out+9,camera_information.flags,2);
    AP_Module::BindingWire::write(out+11,camera_information.resolution_h,2);
    AP_Module::BindingWire::write(out+13,camera_information.resolution_v,2);
    AP_Module::BindingWire::write(out+15,camera_information.rotation,2);
    AP_Module::BindingWire::write(out+17,camera_information.hfov,2);
    out[19]=camera_information.stream_id;out[20]=camera_information.count;out[21]=camera_information.type;
    memcpy(out+22,camera_information.name,32);memcpy(out+54,camera_information.uri,160);
    out[214]=camera_information.encoding;
}
extern "C" void read_route(uint8_t *out) {
    out[0]=route_backend;out[1]=route_iface;AP_Module::BindingWire::write(out+2,route_dest,4);out[6]=route_mask;
}
''']
    source.write_text('\n'.join(lines))
    enabled = int(request.param)
    subprocess.run([
        os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
        f'-DAP_GPS_ENABLED={enabled}', f'-DAP_CAMERA_INFO_FROM_SCRIPT_ENABLED={enabled}',
        f'-DAP_CAMERA_ENABLED={enabled}', f'-DAP_CAMERA_SCRIPTING_ENABLED={enabled}',
        f'-DAP_NETWORKING_ENABLED={enabled}', f'-DAP_SCRIPTING_LUA_ENABLED={enabled}',
        '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source), '-o', str(library),
    ], check=True)
    lib = ctypes.CDLL(str(library))
    for name in ('epoch_call', 'inject_call', 'camera_call', 'route_call', 'restart_call'):
        function = getattr(lib, name)
        function.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
        function.restype = ctypes.c_int
    lib.configure.argtypes = [ctypes.c_bool] * 6
    lib.injected_byte.argtypes = [ctypes.c_uint]
    lib.injected_byte.restype = ctypes.c_uint8
    lib.read_camera.argtypes = [ctypes.c_void_p]
    lib.read_route.argtypes = [ctypes.c_void_p]
    return lib, request.param


def test_host_gps_exact_u64_bounded_injection_and_validation(host):
    lib, enabled = host
    lib.configure(True, True, True, True, True, True)
    output = ctypes.create_string_buffer(8)
    assert lib.epoch_call(b'\1', 1, output, 8) == (0 if enabled else -3)
    assert output.raw == (struct.pack('<Q', 0xfedcba9876543211) if enabled else bytes(8))
    assert lib.epoch_call(b'\2', 1, output, 8) == (-1 if enabled else -3)

    packet = struct.pack('<H', 4) + b'\xd3\0\1\xaa' + bytes(1020)
    assert lib.inject_call(packet, len(packet), None, 0) == (0 if enabled else -3)
    assert lib.injected_count() == (1 if enabled else 0)
    if enabled:
        assert lib.injected_size() == 4
        assert bytes(lib.injected_byte(i) for i in range(4)) == b'\xd3\0\1\xaa'
    for bad in [struct.pack('<H', 0) + bytes(1024), struct.pack('<H', 1025) + bytes(1024),
                struct.pack('<H', 1) + b'a' + b'x' + bytes(1022)]:
        assert lib.inject_call(bad, len(bad), None, 0) == -1


def test_host_camera_owned_fields_text_validation_lock_and_availability(host):
    lib, enabled = host
    packet = _packet()
    lib.configure(True, True, True, True, True, True)
    assert len(packet) == applet.VIDEO_STREAM_WIRE_SIZE
    assert lib.camera_call(packet, len(packet), None, 0) == (0 if enabled else -3)
    assert lib.camera_count() == (1 if enabled else 0)
    if enabled:
        copied = ctypes.create_string_buffer(len(packet))
        lib.read_camera(copied)
        assert copied.raw == packet
        assert lib.camera_device_id() == 0

    bad_padding = bytearray(packet)
    bad_padding[25] = 0
    bad_padding[26] = ord('x')
    assert lib.camera_call(bytes(bad_padding), len(packet), None, 0) == -1
    for framerate in (float('nan'), float('inf'), -1.0):
        assert lib.camera_call(_packet(framerate=framerate), len(packet), None, 0) == -1

    lib.configure(True, True, True, True, False, True)
    before = lib.camera_count()
    assert lib.camera_call(packet, len(packet), None, 0) == (0 if enabled else -3)
    assert lib.camera_count() == before + int(enabled)
    lib.configure(True, False, True, True, True, True)
    assert lib.camera_call(packet, len(packet), None, 0) == -3


def test_host_route_and_lua_restart_effects(host):
    lib, enabled = host
    lib.configure(True, True, True, True, True, False)
    packet = bytes([1, 7]) + struct.pack('<I', 0x01020304) + bytes([32])
    output = ctypes.create_string_buffer(1)
    assert lib.route_call(packet, 7, output, 1) == (0 if enabled else -3)
    assert output.raw == b'\0'
    assert lib.route_count() == (1 if enabled else 0)
    if enabled:
        copied = ctypes.create_string_buffer(7)
        lib.read_route(copied)
        assert copied.raw == packet
    assert lib.route_call(packet[:-1] + b'!', 7, output, 1) == -1

    before = lib.restart_count()
    assert lib.restart_call(None, 0, None, 0) == (0 if enabled else -3)
    assert lib.restart_count() == before + (1 if enabled else 0)
    lib.configure(True, True, True, False, True, True)
    assert lib.restart_call(None, 0, None, 0) == -3


def test_emitted_rust_sdk_wire_validation_and_restart_semantics(tmp_path):
    generated = applet.rust(applet.specification(700), '')
    source = tmp_path / 'applet.rs'
    source.write_text(r'''
use core::ffi::c_void;
#[derive(Clone,Copy,Debug,Eq,PartialEq)] pub enum Error { Invalid, Unavailable, Host }
pub type Result<T=()> = core::result::Result<T,Error>;
fn result(status:i32)->Result {
 match status {0=>Ok(()),-1=>Err(Error::Invalid),-3=>Err(Error::Unavailable),_=>Err(Error::Host)}
}
struct HostApi { binding_call: Option<unsafe extern "C" fn(*mut c_void,u32,*const u8,u32,*mut u8,u32)->i32> }
pub struct Context<'a>{api:&'a HostApi,raw:*mut c_void}
pub mod bindings {
 use super::Context;
 pub struct Gps<'context,'api>{pub(crate) context:&'context Context<'api>}
 pub struct Camera<'context,'api>{pub(crate) context:&'context Context<'api>}
 pub struct Networking<'context,'api>{pub(crate) context:&'context Context<'api>}
 pub struct Scripting<'context,'api>{pub(crate) context:&'context Context<'api>}
}
impl<'api> Context<'api> {
 pub fn gps(&self)->bindings::Gps<'_,'api>{bindings::Gps{context:self}}
 pub fn camera(&self)->bindings::Camera<'_,'api>{bindings::Camera{context:self}}
 pub fn networking(&self)->bindings::Networking<'_,'api>{bindings::Networking{context:self}}
 pub fn scripting(&self)->bindings::Scripting<'_,'api>{bindings::Scripting{context:self}}
}
pub(crate) struct Import(u32); impl Import {pub(crate) fn slot(&self)->u32{self.0}}
pub(crate) mod imports {use super::Import;
pub(crate) static BINDING_700:Import=Import(700); pub(crate) static BINDING_701:Import=Import(701);
pub(crate) static BINDING_702:Import=Import(702); pub(crate) static BINDING_703:Import=Import(703);
pub(crate) static BINDING_704:Import=Import(704);}
mod generated {
''' + generated + r'''
}
#[cfg(test)] mod tests {
use super::{generated::VideoStreamInformation,Context,Error,HostApi}; use core::ffi::c_void;
struct Host { id:u32, input:[u8;1026], ni:usize, calls:u32, route:u8, epoch:u64, status:i32 }
unsafe extern "C" fn call(raw:*mut c_void,id:u32,input:*const u8,ni:u32,output:*mut u8,no:u32)->i32 {
 let host=&mut *raw.cast::<Host>(); host.id=id;host.ni=ni as usize;host.calls+=1;
 if ni!=0 {host.input[..ni as usize].copy_from_slice(core::slice::from_raw_parts(input,ni as usize));}
 if id==700 {assert_eq!(no,8);core::slice::from_raw_parts_mut(output,8).copy_from_slice(&host.epoch.to_le_bytes());}
 if id==703 {assert_eq!(no,1);*output=host.route;}
 host.status
}
fn with<T>(host:&mut Host,f:impl FnOnce(&Context<'_>)->T)->T {
 let api=HostApi{binding_call:Some(call)};let context=Context{api:&api,raw:(host as *mut Host).cast()};f(&context)
}
fn host()->Host {Host{id:0,input:[0;1026],ni:0,calls:0,route:1,epoch:0xfedcba9876543210,status:0}}
#[test] fn gps_wire_and_bounds() {
 let mut h=host();assert_eq!(with(&mut h,|c|c.gps().time_epoch_usec(2)),Ok(h.epoch));
 assert_eq!((h.id,h.ni,h.input[0]),(700,1,2));
 assert_eq!(with(&mut h,|c|c.gps().inject_data(&[0xd3,1,2])),Ok(()));
 assert_eq!((h.id,h.ni), (701,1026));assert_eq!(&h.input[..5],&[3,0,0xd3,1,2]);
 assert!(h.input[5..1026].iter().all(|v|*v==0));let calls=h.calls;
 assert_eq!(with(&mut h,|c|c.gps().inject_data(&[])),Err(Error::Invalid));
 assert_eq!(with(&mut h,|c|c.gps().inject_data(&[0;1025])),Err(Error::Invalid));assert_eq!(h.calls,calls);
}
#[test] fn camera_owned_exact_fields_and_validation() {
 let mut h=host();let mut v=VideoStreamInformation::default();
 v.framerate=29.5;v.bitrate=8_000_000;v.flags=0x1234;v.resolution_h=1920;v.resolution_v=1080;
 v.rotation=270;v.hfov=95;v.stream_id=2;v.count=4;v.stream_type=1;v.encoding=2;
 assert_eq!(v.set_name("main"),Ok(()));assert_eq!(v.set_uri("rtsp://camera/live"),Ok(()));
 assert_eq!(with(&mut h,|c|c.camera().set_stream_information(3,&v)),Ok(()));
 assert_eq!((h.id,h.ni),(702,215));assert_eq!(&h.input[..22],
  &[3,0,0,236,65,0,18,122,0,52,18,128,7,56,4,14,1,95,0,2,4,1]);
 assert_eq!(&h.input[22..27],b"main\0");assert_eq!(&h.input[54..73],b"rtsp://camera/live\0");
 assert_eq!(h.input[214],2);
 assert_eq!(v.set_name("embedded\0nul"),Err(Error::Invalid));
 assert_eq!(v.set_uri(&"x".repeat(161)),Err(Error::Invalid));let calls=h.calls;
 v.framerate=f32::NAN;assert_eq!(with(&mut h,|c|c.camera().set_stream_information(0,&v)),Err(Error::Invalid));
 assert_eq!(h.calls,calls);
}
#[test] fn route_and_restart_wire() {
 let mut h=host();assert_eq!(with(&mut h,|c|c.networking().add_route(1,7,0x01020304,32)),Ok(true));
 assert_eq!((h.id,h.ni),(703,7));assert_eq!(&h.input[..7],&[1,7,4,3,2,1,32]);let calls=h.calls;
 assert_eq!(with(&mut h,|c|c.networking().add_route(0,0,0,33)),Err(Error::Invalid));assert_eq!(h.calls,calls);
 h.route=2;assert_eq!(with(&mut h,|c|c.networking().add_route(0,0,0,0)),Err(Error::Host));
 assert_eq!(with(&mut h,|c|c.scripting().restart_all()),Ok(()));assert_eq!((h.id,h.ni),(704,0));
}
}
''')
    binary = tmp_path / 'applet_tests'
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--test', str(source), '-o', str(binary)],
                   check=True)
    subprocess.run([str(binary), '--quiet'], check=True)


def test_cpp_and_rust_emission_shape():
    spec = applet.specification(700)
    cpp = '\n'.join(applet.cpp(spec))
    rust = applet.rust(spec, '')
    assert 'BindingWire::write(output + 4, uint32_t(value >> 32), 4)' in cpp
    assert 'BindingLock lock(camera->get_semaphore())' in cpp
    assert '#if AP_SCRIPTING_LUA_ENABLED' in cpp
    assert applet.cpp_headers() == [
        '#if AP_SCRIPTING_LUA_ENABLED', '#include <AP_Scripting/AP_Scripting.h>', '#endif']
    assert 'pub struct VideoStreamInformation' in rust
    assert "impl crate::bindings::Scripting<'_, '_>" in rust
    assert 'pub fn restart_all(&self) -> Result' in rust
    assert 'does not stop or restart this Native module' in rust

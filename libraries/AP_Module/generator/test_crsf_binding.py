# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate owned CRSF menu handles, events, responses, and cleanup."""

import copy
import ctypes
import os
import subprocess

import bindings
import crsf_binding as crsf
import pytest


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert crsf.select(ir, 800) == crsf.specification(800)
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['singletons'] if item['name'] == crsf.OWNER)
    alias = next(item for item in owner['aliases'] if item['alias'] == 'peek_menu_event')
    alias['manual'] = False
    with pytest.raises(ValueError, match='unaudited AP_CRSF_Telem.peek_menu_event'):
        crsf.select(changed, 800)
    changed = copy.deepcopy(ir)
    parameter = next(item for item in changed['userdata'] if item['name'] == crsf.PARAMETER)
    method = next(item for item in parameter['methods'] if item['name'] == 'id')
    method['return']['pointer'] = True
    with pytest.raises(ValueError, match='unaudited CRSFParameter.id'):
        crsf.select(changed, 800)


def test_contract_is_pointer_free_bounded_and_lifecycle_owned():
    methods = crsf.specification(800)['methods']
    assert [method['id'] for method in methods] == list(range(800, 813))
    assert [method['name'] for method in methods] == [
        'crsf.add_menu', 'CRSFMenu.add_menu', 'CRSFMenu.add_parameter',
        'CRSFParameter.id', 'CRSFParameter.data', 'crsf.peek_menu_event',
        'crsf.pop_menu_event', 'crsf.send_write_response', 'crsf.send_response',
        'crsf.get_menu_event', 'CRSFMenu.id', 'CRSFMenu.name', 'CRSFMenu.num_params',
    ]
    policy = methods[0]['policy']
    assert 'module-context-owned-menus' in policy
    assert 'no-pointer-wire' in policy
    assert 'queued-pointer-prune-before-free' in policy
    assert 'lua-restart-isolation' in policy


def test_emitted_rust_sdk_wire_and_validation(tmp_path):
    generated = crsf.rust(crsf.specification(800), '')
    source = tmp_path / 'crsf.rs'
    source.write_text(r'''
use core::ffi::c_void;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error { Invalid, Host }
pub type Result<T = ()> = core::result::Result<T, Error>;
fn result(status: i32) -> Result { if status == 0 { Ok(()) } else { Err(Error::Host) } }
struct HostApi {
    binding_call: Option<unsafe extern "C" fn(*mut c_void, u32, *const u8, u32, *mut u8, u32) -> i32>,
}
pub struct Context<'a> { api: &'a HostApi, raw: *mut c_void }
pub mod bindings {
    pub struct Crsf<'context, 'api> {
        pub(crate) context: &'context crate::Context<'api>,
    }
}
impl<'api> Context<'api> {
    pub fn crsf(&self) -> bindings::Crsf<'_, 'api> {
        bindings::Crsf { context: self }
    }
}
pub(crate) struct Import(u32);
impl Import { pub(crate) fn slot(&self) -> u32 { self.0 } }
pub(crate) mod imports {
    use super::Import;
    pub(crate) static BINDING_800: Import = Import(800);
    pub(crate) static BINDING_801: Import = Import(801);
    pub(crate) static BINDING_802: Import = Import(802);
    pub(crate) static BINDING_803: Import = Import(803);
    pub(crate) static BINDING_804: Import = Import(804);
    pub(crate) static BINDING_805: Import = Import(805);
    pub(crate) static BINDING_806: Import = Import(806);
    pub(crate) static BINDING_807: Import = Import(807);
    pub(crate) static BINDING_808: Import = Import(808);
    pub(crate) static BINDING_809: Import = Import(809);
    pub(crate) static BINDING_810: Import = Import(810);
    pub(crate) static BINDING_811: Import = Import(811);
    pub(crate) static BINDING_812: Import = Import(812);
}
mod generated {
''' + generated + r'''
}

#[cfg(test)]
mod tests {
    use super::{generated::*, Context, Error, HostApi};
    use core::ffi::c_void;

    struct Host { id: u32, calls: u32, input: [u8; 260], input_len: usize,
                  output: [u8; 256] }
    unsafe extern "C" fn call(raw: *mut c_void, id: u32, input: *const u8, ni: u32,
                               output: *mut u8, no: u32) -> i32 {
        let host = &mut *raw.cast::<Host>();
        host.id = id; host.calls += 1; host.input_len = ni as usize;
        if ni != 0 { host.input[..ni as usize].copy_from_slice(core::slice::from_raw_parts(input, ni as usize)); }
        if no != 0 { core::slice::from_raw_parts_mut(output, no as usize).copy_from_slice(&host.output[..no as usize]); }
        0
    }
    fn clear(host: &mut Host) { host.output.fill(0); }

    #[test]
    fn exact_wire_owned_handles_and_events() {
        let mut host = Host { id: 0, calls: 0, input: [0; 260], input_len: 0, output: [0; 256] };
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        host.output[..4].copy_from_slice(&7u32.to_le_bytes());
        let menu = context.crsf().add_menu("Calibrate").unwrap().unwrap();
        assert_eq!(host.id, 800);
        assert_eq!(&host.input[..11], &[9, b'C', b'a', b'l', b'i', b'b', b'r', b'a', b't', b'e', 0]);

        clear(&mut host); host.output[0] = 40;
        assert_eq!(menu.id(&context), Ok(40));
        assert_eq!(host.id, 810);
        clear(&mut host); host.output[..10].copy_from_slice(&[9, b'C', b'a', b'l', b'i', b'b', b'r', b'a', b't', b'e']);
        assert_eq!(menu.name(&context).unwrap().as_str(), Ok("Calibrate"));
        assert_eq!(host.id, 811);
        clear(&mut host); host.output[0] = 6;
        assert_eq!(menu.num_params(&context), Ok(6));
        assert_eq!(host.id, 812);

        clear(&mut host); host.output[..4].copy_from_slice(&9u32.to_le_bytes());
        let parameter = menu.add_parameter(&context, &[13, 1, 2]).unwrap().unwrap();
        assert_eq!((host.id, host.input_len), (802, 260));
        assert_eq!(&host.input[..8], &[7, 0, 0, 0, 3, 13, 1, 2]);

        clear(&mut host); host.output[0] = 42;
        assert_eq!(parameter.id(&context), Ok(42));
        assert_eq!(host.id, 803);
        clear(&mut host); host.output[..4].copy_from_slice(&[3, b'a', b'b', b'c']);
        assert_eq!(parameter.data(&context).unwrap().bytes(), b"abc");

        clear(&mut host);
        host.output[..6].copy_from_slice(&[1, 2, 42, 2, 1, 6]);
        let event = context.crsf().peek_menu_event().unwrap().unwrap();
        assert_eq!((event.queue_depth, event.param_id, event.events, event.payload()), (2, 42, 2, &[6][..]));
        clear(&mut host); host.output[0] = 1;
        assert_eq!(context.crsf().pop_menu_event(&event), Ok(true));
        assert_eq!((host.id, &host.input[..2]), (806, &[42, 2][..]));
        clear(&mut host); host.output[0] = 1;
        assert_eq!(context.crsf().send_write_response(&[3, 4]), Ok(true));
        assert_eq!((host.id, host.input[0], &host.input[1..3]), (807, 2, &[3, 4][..]));

        clear(&mut host); host.output[..5].copy_from_slice(&[1, 42, 2, 1, 7]);
        let filtered = context.crsf().get_menu_event(2).unwrap().unwrap();
        assert_eq!((host.id, host.input[0], filtered.param_id, filtered.events, filtered.payload()),
                   (809, 2, 42, 2, &[7][..]));

        let calls = host.calls;
        assert_eq!(menu.add_parameter(&context, &[]), Err(Error::Invalid));
        assert_eq!(context.crsf().add_menu(&"x".repeat(16)), Err(Error::Invalid));
        assert_eq!(host.calls, calls);
    }
}
''')
    subprocess.run(['rustc', '--edition=2021', '--test', str(source), '-o', str(tmp_path / 'test')], check=True)
    subprocess.run([str(tmp_path / 'test')], check=True)


def _write_headers(tmp_path, enabled):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '#pragma once\n#define AP_MODULE_ENABLED 1\n',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { class Semaphore { public: bool take(uint32_t) { return true; } void give() {} }; }
using HAL_Semaphore = AP_HAL::Semaphore;
#define WITH_SEMAPHORE(sem) (void)(sem)
''',
        'AP_Common/AP_Common.h': '''#pragma once
#include <stddef.h>
void *mem_realloc(void *pointer, size_t old_size, size_t new_size);
''',
        'BindingConfig.h': '#pragma once\n',
    }
    if enabled:
        headers['AP_RCTelemetry/AP_CRSF_Telem.h'] = r'''#pragma once
#include <AP_HAL/Semaphores.h>
#include <stdint.h>
#include <string.h>
#define AP_CRSF_SCRIPTING_ENABLED 1
template <typename T, uint8_t N=8> class ObjectBuffer {
public:
    bool push(const T &v) { if (count == N) return false; values[(head+count)%N]=v; count++; return true; }
    bool pop(T &v) { if (!count) return false; v=values[head]; head=(head+1)%N; count--; return true; }
    bool peek(T &v) { if (!count) return false; v=values[head]; return true; }
    uint32_t available() const { return count; }
    T values[N]{}; uint8_t head{}, count{};
};
class AP_CRSF_Telem {
public:
    static constexpr uint8_t MAX_SCRIPTED_MENU_SIZE=20;
    enum ScriptedParameterEvents : uint8_t { PARAMETER_READ=1, PARAMETER_WRITE=2 };
    struct ScriptedParameter { uint8_t id{}, parent_id{}; uint16_t length{}; const char *data{}; };
    struct ScriptedMenu {
        uint8_t id{}, parent_id{}, num_params{}; const char *name{}; ScriptedParameter *params{};
        ScriptedMenu *next_menu{}; const void *owner{};
        ScriptedMenu *add_menu(const char *, uint8_t, uint8_t, const void *);
        ScriptedParameter *add_parameter(uint8_t, const char *);
    } scripted_menus;
    struct ScriptedPayload { uint8_t payload_length{}; uint8_t payload[57]{}; };
    struct ScriptedParameterWrite { ScriptedParameterEvents type{}; uint8_t destination{}, origin{}, param_num{},
        param_chunk{}; ScriptedParameter *param{}; ScriptedPayload payload{}; };
    ObjectBuffer<ScriptedParameterWrite> inbound_params, outbound_params, ready_params;
    bool select_scripted_event(ObjectBuffer<ScriptedParameterWrite> &queue, const void *owner,
                                ScriptedParameterWrite &request) {
        const unsigned count=queue.available();
        for(unsigned i=0;i<count && queue.peek(request);i++) {
            for(auto *menu=scripted_menus.next_menu;menu;menu=menu->next_menu) {
                if(menu->owner!=owner) continue;
                for(uint8_t p=0;p<menu->num_params;p++) if(&menu->params[p]==request.param) return true;
            }
            queue.pop(request); queue.push(request);
        }
        return false;
    }
    HAL_Semaphore sem;
    HAL_Semaphore &get_semaphore() { return sem; }
    ScriptedMenu *add_menu(const char *, const void *);
    void clear_menus(const void *);
};
namespace AP { AP_CRSF_Telem *crsf_telem(); }
'''
    else:
        headers['AP_RCTelemetry/AP_CRSF_Telem.h'] = '#pragma once\n#define AP_CRSF_SCRIPTING_ENABLED 0\n'
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)


@pytest.fixture
def crsf_host(tmp_path):
    _write_headers(tmp_path, True)
    source = tmp_path / 'wrapper.cpp'
    source.write_text(r'''
#include <AP_RCTelemetry/AP_CRSF_Telem.h>
#include "BindingCRSF.h"
#include <stdlib.h>
#include <string.h>
static AP_CRSF_Telem crsf;
static uint8_t next_id=40;
void *mem_realloc(void *pointer, size_t, size_t size) { return realloc(pointer, size); }
namespace AP { AP_CRSF_Telem *crsf_telem() { return &crsf; } }
AP_CRSF_Telem::ScriptedMenu *AP_CRSF_Telem::ScriptedMenu::add_menu(
    const char *new_name, uint8_t, uint8_t parent, const void *new_owner) {
    ScriptedMenu *tail=this; while (tail->next_menu) tail=tail->next_menu;
    auto *menu=new ScriptedMenu{}; menu->id=next_id++; menu->parent_id=parent; menu->owner=new_owner;
    menu->name=strdup(new_name); tail->next_menu=menu;
    if (parent != 0) { auto *link=add_parameter(0, nullptr); link->id=menu->id; }
    return menu;
}
AP_CRSF_Telem::ScriptedParameter *AP_CRSF_Telem::ScriptedMenu::add_parameter(uint8_t length, const char *value) {
    auto *next=(ScriptedParameter*)realloc(params, sizeof(*params)*(num_params+1)); if (!next) return nullptr;
    params=next; auto &parameter=params[num_params++]; parameter={uint8_t(id+num_params),id,length,nullptr};
    if (length != 0) {
        parameter.data=(const char*)malloc(length); memcpy((char*)parameter.data,value,length);
    }
    return &parameter;
}
AP_CRSF_Telem::ScriptedMenu *AP_CRSF_Telem::add_menu(const char *name, const void *owner) {
    return scripted_menus.add_menu(name,2,0,owner);
}
static bool contains(AP_CRSF_Telem::ScriptedMenu *menu, AP_CRSF_Telem::ScriptedParameter *parameter) {
    for (uint8_t i=0;i<menu->num_params;i++) {
        if (&menu->params[i]==parameter) {
            return true;
        }
    }
    return false;
}
void AP_CRSF_Telem::clear_menus(const void *owner) {
    auto prune=[&](auto &queue) { unsigned count=queue.available(); while (count--) {
        ScriptedParameterWrite item{}; queue.pop(item); bool owned=false;
        for (auto *m=scripted_menus.next_menu;m;m=m->next_menu) if (m->owner==owner && contains(m,item.param)) owned=true;
        if (!owned) queue.push(item);
    }}; prune(inbound_params); prune(outbound_params); prune(ready_params);
    auto *previous=&scripted_menus; while (previous->next_menu) { auto *menu=previous->next_menu;
        if (menu->owner==owner) { previous->next_menu=menu->next_menu;
            for(uint8_t i=0;i<menu->num_params;i++) free((void*)menu->params[i].data);
            free(menu->params); free((void*)menu->name); delete menu; } else previous=menu; }
}
extern "C" int32_t call(uint8_t method, void *context, const uint8_t *input, uint32_t size,
                         uint8_t *output, uint32_t output_size) {
    using namespace AP_Module;
    switch(method) { case 0:return binding_crsf_add_menu(context,input,size,output,output_size);
    case 1:return binding_crsf_menu_add_menu(context,input,size,output,output_size);
    case 2:return binding_crsf_menu_add_parameter(context,input,size,output,output_size);
    case 3:return binding_crsf_parameter_id(context,input,size,output,output_size);
    case 4:return binding_crsf_parameter_data(context,input,size,output,output_size);
    case 5:return binding_crsf_peek_menu_event(context,input,size,output,output_size);
    case 6:return binding_crsf_pop_menu_event(context,input,size,output,output_size);
    case 7:return binding_crsf_send_write_response(context,input,size,output,output_size);
    case 8:return binding_crsf_send_response(context,input,size,output,output_size);
    case 9:return binding_crsf_get_menu_event(context,input,size,output,output_size);
    case 10:return binding_crsf_menu_id(context,input,size,output,output_size);
    case 11:return binding_crsf_menu_name(context,input,size,output,output_size);
    default:return binding_crsf_menu_num_params(context,input,size,output,output_size); }
}
extern "C" void cleanup(void *context) { AP_Module::binding_crsf_cleanup(context); }
extern "C" bool inject(uint8_t id, uint8_t type, uint8_t payload) {
    for(auto *m=crsf.scripted_menus.next_menu;m;m=m->next_menu) {
        for(uint8_t i=0;i<m->num_params;i++) {
            if(m->params[i].id==id) {
                AP_CRSF_Telem::ScriptedParameterWrite item{}; item.type=(AP_CRSF_Telem::ScriptedParameterEvents)type;
                item.param=&m->params[i]; item.payload.payload_length=1; item.payload.payload[0]=payload;
                return crsf.inbound_params.push(item);
            }
        }
    }
    return false;
}
extern "C" uint8_t ready_id() {
    AP_CRSF_Telem::ScriptedParameterWrite event{};
    return crsf.ready_params.peek(event) ? event.param->id : 0;
}
extern "C" uint8_t menus() { uint8_t count=0;
    for(auto *m=crsf.scripted_menus.next_menu;m;m=m->next_menu) { count++; }
    return count; }
''')
    library = tmp_path / 'crsf.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source),
                    str(bindings.ROOT / 'BindingCRSF.cpp'), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.call.argtypes = [ctypes.c_uint8, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32,
                         ctypes.c_void_p, ctypes.c_uint32]
    lib.cleanup.argtypes = [ctypes.c_void_p]
    lib.inject.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8]
    return lib


def _name(value):
    encoded = value.encode()
    return bytes([len(encoded)]) + encoded + bytes(15 - len(encoded))


def _parameter(token, value):
    return token.to_bytes(4, 'little') + bytes([len(value)]) + value + bytes(255 - len(value))


def test_host_tokens_event_ownership_responses_and_cleanup(crsf_host):
    lib = crsf_host
    owner_a, owner_b = ctypes.c_uint8(), ctypes.c_uint8()
    a, b = ctypes.addressof(owner_a), ctypes.addressof(owner_b)
    output = ctypes.create_string_buffer(256)
    assert lib.call(0, a, _name('A'), 16, output, 4) == 0
    menu_a = int.from_bytes(output.raw[:4], 'little')
    assert menu_a != 0
    assert lib.call(10, a, menu_a.to_bytes(4, 'little'), 4, output, 1) == 0
    assert output.raw[0] == 40
    assert lib.call(11, a, menu_a.to_bytes(4, 'little'), 4, output, 16) == 0
    assert output.raw[:2] == b'\1A'
    assert lib.call(12, a, menu_a.to_bytes(4, 'little'), 4, output, 1) == 0
    assert output.raw[0] == 0
    assert lib.call(2, b, _parameter(menu_a, b'bad'), 260, output, 4) == -1
    assert lib.call(2, a, _parameter(menu_a, b'abc'), 260, output, 4) == 0
    parameter_a = int.from_bytes(output.raw[:4], 'little')
    assert lib.call(12, a, menu_a.to_bytes(4, 'little'), 4, output, 1) == 0
    assert output.raw[0] == 1
    assert lib.call(3, a, parameter_a.to_bytes(4, 'little'), 4, output, 1) == 0
    id_a = output.raw[0]
    assert lib.call(4, a, parameter_a.to_bytes(4, 'little'), 4, output, 256) == 0
    assert output.raw[:4] == b'\3abc'

    assert lib.call(0, b, _name('B'), 16, output, 4) == 0
    menu_b = int.from_bytes(output.raw[:4], 'little')
    assert lib.call(2, b, _parameter(menu_b, b'def'), 260, output, 4) == 0
    parameter_b = int.from_bytes(output.raw[:4], 'little')
    assert lib.call(3, b, parameter_b.to_bytes(4, 'little'), 4, output, 1) == 0
    id_b = output.raw[0]
    assert lib.inject(id_b, 2, 6)
    assert lib.call(5, a, None, 0, output, 62) == 0 and output.raw[0] == 0
    assert lib.call(5, b, None, 0, output, 62) == 0
    assert output.raw[:6] == bytes([1, 1, id_b, 2, 1, 6])
    assert lib.call(6, a, bytes([id_b, 2]), 2, output, 1) == 0 and output.raw[0] == 0
    assert lib.call(6, b, bytes([id_b, 2]), 2, output, 1) == 0 and output.raw[0] == 1
    assert lib.call(8, a, None, 0, output, 1) == 0 and output.raw[0] == 0
    assert lib.call(8, b, None, 0, output, 1) == 0 and output.raw[0] == 1

    assert lib.inject(id_b, 2, 7)
    assert lib.call(9, a, b'\2', 1, output, 61) == 0 and output.raw[0] == 0
    assert lib.call(9, b, b'\2', 1, output, 61) == 0
    assert output.raw[:5] == bytes([1, id_b, 2, 1, 7])
    assert lib.call(8, b, None, 0, output, 1) == 0 and output.raw[0] == 1

    assert lib.inject(id_a, 1, 0)
    assert lib.call(5, a, None, 0, output, 62) == 0
    assert lib.call(6, a, bytes([id_a, 1]), 2, output, 1) == 0 and output.raw[0] == 1
    response = bytes([2, 9, 8]) + bytes(253)
    assert lib.call(7, a, response, 256, output, 1) == 0 and output.raw[0] == 1
    lib.cleanup(a)
    assert lib.menus() == 1
    assert lib.call(3, a, parameter_a.to_bytes(4, 'little'), 4, output, 1) == -1
    assert lib.call(3, b, parameter_b.to_bytes(4, 'little'), 4, output, 1) == 0


def test_host_compiles_with_crsf_scripting_disabled(tmp_path):
    _write_headers(tmp_path, False)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsyntax-only',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
                    str(bindings.ROOT / 'BindingCRSF.cpp')], check=True)


def test_core_cleanup_is_owner_selective_and_prunes_before_delete():
    header = (bindings.ROOT.parent / 'AP_RCTelemetry/AP_CRSF_Telem.h').read_text()
    source = (bindings.ROOT.parent / 'AP_RCTelemetry/AP_CRSF_Telem.cpp').read_text()
    assert 'const void* owner' in header
    cleanup = source.index('void AP_CRSF_Telem::clear_menus(const void* owner)')
    prune = source.index('prune(inbound_params);', cleanup)
    deletion = source.index('delete menu;', prune)
    assert prune < deletion
    assert 'menu->owner == owner' in source[cleanup:deletion + len('delete menu;')]


@pytest.mark.parametrize('method', [5, 6, 9])
def test_submenu_navigation_does_not_block_later_events(crsf_host, method):
    lib = crsf_host
    owner = ctypes.c_void_p(51)
    output = ctypes.create_string_buffer(256)
    assert lib.call(0, owner, _name('root'), 16, output, 4) == 0
    root = output.raw[:4]
    assert lib.call(1, owner, root + _name('child'), 20, output, 4) == 0
    child = output.raw[:4]
    assert lib.call(10, owner, child, 4, output, 1) == 0
    child_id = output.raw[0]
    assert lib.call(2, owner, _parameter(int.from_bytes(child, 'little'), b'data'), 260, output, 4) == 0
    parameter = output.raw[:4]
    assert lib.call(3, owner, parameter, 4, output, 1) == 0
    parameter_id = output.raw[0]
    assert lib.inject(child_id, 2, 1)
    assert lib.inject(parameter_id, 2, 7)
    if method == 5:
        assert lib.call(5, owner, None, 0, output, 62) == 0
        assert output.raw[0] == 1 and output.raw[2] == parameter_id
    elif method == 6:
        assert lib.call(6, owner, bytes([parameter_id, 2]), 2, output, 1) == 0
        assert output.raw[0] == 1
    else:
        assert lib.call(9, owner, b'\2', 1, output, 61) == 0
        assert output.raw[0] == 1 and output.raw[1] == parameter_id
    assert lib.ready_id() == child_id
    lib.cleanup(owner)

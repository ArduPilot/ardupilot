# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise generic bounded packets in independently compiled Rust and C++."""

import json
import os
import subprocess

import call_model as model
import contracts
import cpp_bindings
import generate
import pytest
import rust_calls


@pytest.fixture
def bounded_model():
    field = model.Field
    send = model.Buffer(4)
    receive = model.Buffer(4, 'out')
    return model.Module((
        model.Call('sample.exchange', 1, (field('send', send), field('receive', receive)),
                   model.BufferResult('receive', 4), request=(field('send_length', model.Length('send', model.U8)),
                   field('receive_length', model.Length('receive', model.U8)), field('send', send))),
        model.Call('sample.read', 2, (field('buffer', model.Buffer(4, 'out', empty_returns_zero=True)),),
                   model.BufferResult('buffer', 4, model.U16),
                   request=(field('length', model.Length('buffer', model.U16)),)),
        model.Call('sample.write', 3, (field('buffer', model.Buffer(4, empty_returns_zero=True)),),
                   model.BoundedCount(model.U16, 'buffer'),
                   request=(field('length', model.Length('buffer', model.U8)),
                            field('buffer', model.Buffer(4, empty_returns_zero=True)))),
        model.Call('sample.start', 4, (field('name', model.String(8)), field('other', model.String(8))), model.BOOL),
        model.Call('sample.baud', 5, (field('baud', model.OptionalValue(model.U32, nonzero=True)),)),
    ))


def test_buffer_model_rejects_inconsistent_packets():
    field = model.Field
    for capacity, minimum in ((0, 0), (4, -1), (4, 5), (4, 0.5)):
        with pytest.raises(ValueError, match='bounded buffer'):
            model.Buffer(capacity, minimum=minimum)
    with pytest.raises(ValueError, match='buffer length input'):
        model.Call('sample.bad', 1, (field('send', model.Buffer(256)),),
                   request=(field('length', model.Length('send', model.U8)), field('send', model.Buffer(256))))
    with pytest.raises(ValueError, match='matching output buffer'):
        model.Call('sample.bad', 1, (field('send', model.Buffer(4)),), model.BufferResult('send', 4))
    with pytest.raises(ValueError, match='empty zero return'):
        model.Call('sample.bad', 1, (field('send', model.Buffer(4, empty_returns_zero=True)),), model.BOOL)
    with pytest.raises(ValueError, match='matching request length'):
        model.Call('sample.bad', 1, (field('send', model.Buffer(4)),), model.BoundedCount(model.U16, 'send'))
    with pytest.raises(ValueError, match='unsupported optional'):
        model.OptionalValue(model.Buffer(4))
    with pytest.raises(ValueError, match='unsupported call result'):
        model.Call('sample.bad', 1, result=model.String(4))


def test_rust_buffer_calls_check_before_copying(tmp_path, bounded_model):
    generated = rust_calls.emit(bounded_model, '')
    source = tmp_path / 'buffers.rs'
    source.write_text('''
#[derive(Debug, PartialEq)] pub enum Error { Host, Invalid, Unavailable }
pub type Result<T=()> = core::result::Result<T, Error>;
pub fn result(v:i32) -> Result { if v==0 { Ok(()) } else { Err(Error::Unavailable) } }
pub struct Api { binding_call: Option<unsafe extern "C" fn(*mut core::ffi::c_void,u32,*const u8,u32,*mut u8,u32)->i32> }
pub struct Context<'a> { raw:*mut core::ffi::c_void, api:&'a Api }
pub mod bindings { pub struct Sample<'c,'a> { pub(crate) context:&'c super::Context<'a> } }
pub mod imports {
    pub struct Import(u32); impl Import { pub fn slot(&self)->u32 {self.0} }
''' + '\n'.join(f'pub static BINDING_{i}:Import=Import({i});' for i in range(1, 6)) + '''
}
mod generated {
''' + generated + '''
}
struct Host { mode:u8, calls:usize }
unsafe extern "C" fn call(raw:*mut core::ffi::c_void,id:u32,i:*const u8,ni:u32,o:*mut u8,no:u32)->i32 {
    let h=&mut *raw.cast::<Host>(); h.calls+=1;
    if h.mode==3 { return -3; }
    let input=if ni==0 {&[]} else {core::slice::from_raw_parts(i,ni as usize)};
    let output=if no==0 {&mut []} else {core::slice::from_raw_parts_mut(o,no as usize)};
    match id {
        1 => { assert_eq!((ni,no),(6,5)); assert_eq!(&input[..2],&[2,3]); assert_eq!(&input[2..],b"ab\\0\\0");
               output[0]=if h.mode==0 {1} else if h.mode==1 {0} else {2}; output[1..].copy_from_slice(b"wxyz"); }
        2 => { assert_eq!((ni,no),(2,6)); assert_eq!(input,3u16.to_le_bytes());
               let length=if h.mode==1 {4u16} else {2}; output[..2].copy_from_slice(&length.to_le_bytes());
               output[2..4].copy_from_slice(b"xy"); if h.mode==2 {output[5]=1;} }
        3 => { assert_eq!((ni,no),(5,2)); assert_eq!(input,&[2,b'a',b'b',0,0]);
               let count=if h.mode==1 {3u16} else {1}; output.copy_from_slice(&count.to_le_bytes()); }
        4 => { assert_eq!((ni,no),(16,1)); assert_eq!(input,&[0xc3,0xa9,0,0,0,0,0,0,b'x',0,0,0,0,0,0,0]);
               output[0]=if h.mode==2 {2} else {1}; }
        5 => { assert_eq!((ni,no),(5,0));
               assert_eq!(input,if h.mode==1 {&[0,0,0,0,0][..]} else {&[1,0x78,0x56,0x34,0x12][..]}); }
        _ => panic!("unknown binding"),
    }
    0
}
#[test] fn exercise() {
    let mut host=Host{mode:0,calls:0}; let api=Api{binding_call:Some(call)};
    let c=Context{raw:(&mut host as *mut Host).cast(),api:&api}; let s=bindings::Sample{context:&c};
    let mut receive=[0x55;3]; assert_eq!(s.exchange(b"ab",&mut receive),Ok(true)); assert_eq!(&receive,b"wxy");
    receive.fill(0x55); host.mode=1; assert_eq!(s.exchange(b"ab",&mut receive),Ok(false));
    assert_eq!(receive,[0x55;3]);
    for mode in [2,3] { host.mode=mode; assert!(s.exchange(b"ab",&mut receive).is_err()); assert_eq!(receive,[0x55;3]); }
    host.mode=0; assert_eq!(s.read(&mut receive),Ok(2)); assert_eq!(receive,[b'x',b'y',0x55]);
    for mode in [1,2,3] { host.mode=mode; receive.fill(0x55);
        assert!(s.read(&mut receive).is_err()); assert_eq!(receive,[0x55;3]); }
    host.mode=0; assert_eq!(s.write(b"ab"),Ok(1)); host.mode=1; assert_eq!(s.write(b"ab"),Err(Error::Host));
    host.mode=0; assert_eq!(s.start("é","x"),Ok(true)); s.baud(Some(0x12345678)).unwrap();
    host.mode=1; s.baud(None).unwrap(); host.mode=2; assert_eq!(s.start("é","x"),Err(Error::Host));
    let before=host.calls; assert_eq!(s.baud(Some(0)),Err(Error::Invalid));
    assert_eq!(s.start("12345678","x"),Err(Error::Invalid)); assert_eq!(s.start("a\\0b","x"),Err(Error::Invalid));
    assert_eq!(s.read(&mut []),Ok(0)); assert_eq!(s.write(&[]),Ok(0));
    assert_eq!(s.read(&mut [0;5]),Err(Error::Invalid)); assert_eq!(s.write(&[0;5]),Err(Error::Invalid));
    assert_eq!(s.exchange(&[0;5],&mut receive),Err(Error::Invalid));
    assert_eq!(host.calls,before);
}
''')
    executable = tmp_path / 'buffers'
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--test',
                    str(source), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_cpp_uses_model_not_contract_wire_strings(tmp_path, monkeypatch, bounded_model):
    output = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), output)
    calls = {call.name: call for call in bounded_model.calls}
    entries = [dict(hash=f'{call.binding_id:016x}', contract=dict(name=call.name)) for call in bounded_model.calls]
    monkeypatch.setattr(model, 'calls', lambda spec: calls)
    monkeypatch.setattr(contracts, 'describe', lambda spec: entries)
    # No serialized argument/result descriptions: every wire type must come
    # from the typed model, not the legacy contract-string parser.
    generated = cpp_bindings.emit(dict(methods=[], math=dict(types=[]), parameter=dict(methods=[])), '')
    generate.write_outputs(generated, output)
    source = tmp_path / 'buffers.cpp'
    source.write_text('''#include <bindings.hpp>
using namespace ap_module;
static unsigned current, calls, mode;
static int32_t call(void *, uint32_t, const uint8_t *p, uint32_t ni, uint8_t *o, uint32_t no) {
    calls++;
    if (current==1) {
        if (ni!=6 || no!=5 || p[0]!=2 || p[1]!=3 || p[2]!='a' || p[3]!='b' || p[4] || p[5]) { return -1; }
        o[0]=1; o[1]='w'; o[2]='x'; o[3]='y'; o[4]='z';
    } else if (current==2) {
        if (ni!=2 || no!=6 || p[0]!=3 || p[1]) { return -1; }
        o[0]=mode==1 ? 5 : mode==2 ? 4 : 2; o[2]='x'; o[3]='y'; if (mode==3) {o[5]=1;}
    } else if (current==3) {
        if (ni!=5 || no!=2 || p[0]!=2 || p[1]!='a' || p[2]!='b' || p[3] || p[4]) { return -1; }
        o[0]=mode==1 ? 3 : 1;
    } else if (current==4) {
        if (ni!=16 || no!=1 || p[0]!=0xc3 || p[1]!=0xa9 || p[8]!='x') { return -1; } o[0]=1;
    } else { return -1; }
    return 0;
}
int main() {
    AP_ModuleHostApi api{}; api.binding_call=call; ContextBase base(api,nullptr); Context c(base);
    Array<uint8_t,4> send{}; send.data[0]='a'; send.data[1]='b';
    current=1; auto exchange=c.sample().exchange(2,3,send);
    if (!exchange || !exchange.value.present || exchange.value.value.data[2]!='y') { return 1; }
    current=2; auto read=c.sample().read(3); if (!read || read.value.length!=2 || read.value.data.data[1]!='y') { return 2; }
    current=3; auto write=c.sample().write(2,send); if (!write || write.value!=1) { return 3; }
    current=4; auto start=c.sample().start(StringView("é"),StringView("x")); if (!start || !start.value) { return 4; }
    if (c.sample().start(StringView("12345678"),StringView("x")).error!=Error::Invalid) { return 5; }
    current=2; for (mode=1; mode<=3; mode++) {if (c.sample().read(3).error!=Error::Host) {return 6;}}
    current=3; mode=1; if (c.sample().write(2,send).error!=Error::Host) {return 7;}
    return calls==8 ? 0 : 8;
}
''')
    executable = tmp_path / 'buffers'
    subprocess.run(['g++', '-std=c++17', '-fsanitize=address,undefined', '-I' + str(generate.ROOT / 'sdk/cpp'),
                    '-I' + str(output), '-I' + str(output / 'sdk/cpp'), str(source),
                    str(output / 'sdk/cpp/imports.cpp'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)

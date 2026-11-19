# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Shared typed layouts and subsystem-independent Rust client generation."""

import os
import subprocess

from pathlib import Path

import bindings
import call_model as model
import pytest
import rust_calls


def test_record_layout_is_component_order_not_compiler_padding():
    value = model.Record('Sample', (model.Field('flag', model.BOOL), model.Field('number', model.U64)))
    assert value.size == 9
    assert [(f.name, p) for f, p in model.layout(value.fields)] == [('flag', 0), ('number', 1)]
    handle = model.Handle('Backend', 'borrowed-index')
    call = model.Call('sensor.open', 7, (model.Field('index', model.U8),), model.SelectedHandle(handle, 'index'))
    assert (call.input_size, call.output_size) == (1, 1)
    assert model.OptionalValue(model.F32).size == 5
    assert model.OptionalValue(model.Handle('Socket', 'closeable-token')).size == 4


def test_invalid_model_is_rejected():
    with pytest.raises(ValueError, match='scalar layout'):
        model.Scalar('u32', 8)
    with pytest.raises(ValueError, match='enum members'):
        model.Enum('Tag', model.U8, (('Overflow', 256),))
    with pytest.raises(ValueError, match='duplicate record'):
        model.Record('Value', (model.Field('x', model.U8), model.Field('x', model.U8)))
    with pytest.raises(ValueError, match='inverted'):
        model.Field('x', model.U8, 7, 2)
    for type, low, high in ((model.U8, -1, 255), (model.I8, -128, 128), (model.F32, 0, 1),
                            (model.F32, 0.0, float('inf')), (model.BOOL, False, True)):
        with pytest.raises(ValueError, match='range'):
            model.Field('x', type, low, high)
    handle = model.Handle('Backend', 'borrowed-index')
    with pytest.raises(ValueError, match='unsupported record field'):
        model.Record('Value', (model.Field('handle', handle),))
    with pytest.raises(ValueError, match='explicit selected input'):
        model.OptionalValue(handle)
    with pytest.raises(ValueError, match='selector must name'):
        model.Call('sensor.open', 7, (model.Field('index', model.U16),), model.SelectedHandle(handle, 'index'))
    with pytest.raises(ValueError, match='close needs'):
        model.Call('Backend.close', 9, receiver=handle, consume=True)
    with pytest.raises(ValueError, match='duplicate modeled binding'):
        model.Module((model.Call('sensor.read', 3), model.Call('sensor.read', 4)))


def test_adapter_models_cover_shared_calls_and_not_source_templates():
    spec = bindings.specification()
    calls = model.calls(spec)
    assert len(calls) == 51
    assert calls['airspeed.get_backend'].output_size == 1
    assert calls['RangeFinderBackend.get_state'].output_size == 13
    assert calls['ProximityBackend.handle_script_3d_msg'].input_size == 14
    assert calls['parameter.old_value'].output_size == 5
    assert (calls['onvif.start'].input_size, calls['onvif.start'].output_size) == (768, 1)
    templates = Path(__file__).parent / 'templates/rust'
    for name in ('airspeed', 'analog_pwm', 'rangefinder_proximity', 'periph', 'serialled',
                 'old_parameter', 'serial', 'i2c', 'onvif'):
        assert not (templates / (name + '.rs.tmpl')).exists()


def test_generated_clients_handle_wire_values_and_bad_replies(tmp_path):
    backend = model.Handle('SampleBackend', 'borrowed-index')
    tag = model.Enum('Quality', model.U8, (('Unknown', 0), ('Good', 1)), default='Unknown')
    reading = model.Record('Reading', (model.Field('number', model.I16), model.Field('value', model.F32),
                                       model.Field('valid', model.BOOL), model.Field('quality', tag)))
    calls = (
        model.Call('sample.get_backend', 1, (model.Field('index', model.U8),), model.SelectedHandle(backend, 'index')),
        model.Call('SampleBackend.update', 2, (model.Field('value', model.F32),), model.BOOL, receiver=backend),
        model.Call('SampleBackend.read', 3, result=reading, receiver=backend),
        model.Call('sample.old', 4, (model.Field('key', model.U16),), model.OptionalValue(model.F32)),
        model.Call('sample.flags', 5, result=model.U64),
        model.Call('sample.bounded', 6, (model.Field('channel', model.U8, 1, 7),), model.BOOL),
    )
    generated = rust_calls.emit(model.Module(calls, (backend, tag, reading)), '')
    source = tmp_path / 'generic.rs'
    source.write_text('''
#[derive(Debug, PartialEq)] pub enum Error { Host, Invalid, Unavailable }
pub type Result<T=()> = core::result::Result<T, Error>;
pub fn result(v:i32) -> Result { if v==0 { Ok(()) } else { Err(Error::Unavailable) } }
pub struct Api { binding_call: Option<unsafe extern "C" fn(*mut core::ffi::c_void,u32,*const u8,u32,*mut u8,u32)->i32> }
pub struct Context<'a> { raw:*mut core::ffi::c_void, api:&'a Api }
pub mod bindings {
    pub struct Sample<'c,'a> { pub(crate) context:&'c super::Context<'a> }
}
pub mod imports {
    pub struct Import(u32); impl Import { pub fn slot(&self)->u32 {self.0} }
''' + '\n'.join(f'pub static BINDING_{i}:Import=Import({i});' for i in range(1, 7)) + '''
}
mod generated {
''' + generated + '''
}
struct Host { mode:u8, calls:usize }
unsafe extern "C" fn call(raw:*mut core::ffi::c_void,id:u32,i:*const u8,ni:u32,o:*mut u8,no:u32)->i32 {
    let h=&mut *raw.cast::<Host>(); h.calls+=1;
    let input=if ni==0 {&[]} else {core::slice::from_raw_parts(i,ni as usize)};
    let output=if no==0 {&mut []} else {core::slice::from_raw_parts_mut(o,no as usize)};
    if h.mode==9 { return -3; }
    match id {
        1 => { assert_eq!(input,&[42]); assert_eq!(no,1); output[0]=h.mode; }
        2 => { assert_eq!((ni,no),(5,1)); assert_eq!(input[0],42);
               assert_eq!(&input[1..],&0x7fc12345u32.to_le_bytes()); output[0]=h.mode; }
        3 => { assert_eq!(input,&[42]); assert_eq!(no,8);
               output[..2].copy_from_slice(&(-1234i16).to_le_bytes());
               output[2..6].copy_from_slice(&0x7fc12345u32.to_le_bytes());
               output[6]=if h.mode==2 {2} else {1}; output[7]=if h.mode==3 {2} else {1}; }
        4 => { assert_eq!(input,&0x1234u16.to_le_bytes()); assert_eq!(no,5); output[0]=h.mode;
               if h.mode==1 {output[1..].copy_from_slice(&0x7fc12345u32.to_le_bytes());}
               if h.mode==3 {output[0]=0;output[1]=1;} }
        5 => { assert_eq!((ni,no),(0,8)); output.copy_from_slice(&0xfedcba9876543210u64.to_le_bytes()); }
        6 => { assert_eq!((ni,no),(1,1)); output[0]=1; }
        _ => panic!("unknown binding"),
    }
    0
}
#[test] fn exercise() {
    let mut host=Host{mode:1,calls:0}; let api=Api{binding_call:Some(call)};
    let c=Context{raw:(&mut host as *mut Host).cast(),api:&api};
    let s=bindings::Sample{context:&c};
    let b=s.get_backend(42).unwrap().unwrap();
    assert_eq!(b.update(&c,f32::from_bits(0x7fc12345)),Ok(true));
    let reading=b.read(&c).unwrap(); assert_eq!(reading.number,-1234);
    assert_eq!(reading.value.to_bits(),0x7fc12345); assert!(reading.valid);
    assert_eq!(reading.quality,generated::Quality::Good);
    assert_eq!(s.flags(),Ok(0xfedcba9876543210));
    assert_eq!(s.old(0x1234).unwrap().unwrap().to_bits(),0x7fc12345);
    host.mode=0; assert_eq!(s.get_backend(42),Ok(None)); assert_eq!(s.old(0x1234),Ok(None));
    host.mode=2; assert_eq!(s.get_backend(42),Err(Error::Host));
    assert_eq!(b.update(&c,f32::from_bits(0x7fc12345)),Err(Error::Host));
    assert!(matches!(b.read(&c),Err(Error::Host)));
    host.mode=3; assert!(matches!(b.read(&c),Err(Error::Host))); assert_eq!(s.old(0x1234),Err(Error::Host));
    let before=host.calls; assert_eq!(s.bounded(0),Err(Error::Invalid)); assert_eq!(host.calls,before);
    host.mode=9; assert_eq!(s.get_backend(42),Err(Error::Unavailable));
}
''')
    executable = tmp_path / 'generic'
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--test',
                    str(source), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_owned_handles_records_and_colliding_helper_names(tmp_path):
    owned = model.Handle('Owned', 'closeable-token')
    channel = model.Handle('Channel', 'context-token')
    tag = model.Enum('Tag', model.I8, (('Good', -1),))  # Deliberately no Default implementation.
    vector = model.Record('ExternalRecord', (model.Field('x', model.F32), model.Field('y', model.F32)), external=True)
    rotation = model.Enum('ExternalEnum', model.I32, external=True)
    value = model.Record('Foo', (model.Field('small', model.I8), model.Field('number', model.I32),
                                 model.Field('mask', model.U64), model.Field('tag', tag), model.Field('vector', vector)))
    named = model.Record('OptionFoo', (model.Field('x', model.U8),))
    calls = (
        model.Call('sample.open', 1, result=model.OptionalValue(owned)),
        model.Call('sample.channel', 2, result=model.OptionalValue(channel)),
        model.Call('Owned.read', 3, result=model.F32, receiver=owned),
        model.Call('Channel.read', 4, result=model.F32, receiver=channel),
        model.Call('Owned.close', 5, receiver=owned, consume=True),
        model.Call('sample.write', 6, (model.Field('value', value), model.Field('rotation', rotation)), model.BOOL),
        model.Call('sample.optional_a', 7, result=model.OptionalValue(value)),
        model.Call('sample.optional_b', 8, result=model.OptionalValue(value)),
        model.Call('sample.named_a', 9, result=named),
        model.Call('sample.named_b', 10, result=named),
        model.Call('sample.bounded', 11, (model.Field('value', model.F32, 1.0, 2.0),), model.BOOL),
        model.Call('sample.rotation', 12, result=rotation),
        model.Call('sample.write_again', 13, (model.Field('value', value), model.Field('rotation', rotation)), model.BOOL),
    )
    generated = rust_calls.emit(model.Module(calls, (owned, channel, tag, vector, rotation, value, named)), '')
    assert 'fn call_empty_option_foo(' in generated
    assert 'fn call_empty_option_foo_2(' in generated
    source = tmp_path / 'owned.rs'
    program = '''
#[derive(Debug, PartialEq)] pub enum Error { Host, Invalid, Unavailable }
pub type Result<T=()> = core::result::Result<T, Error>;
pub fn result(v:i32) -> Result { if v==0 { Ok(()) } else { Err(Error::Unavailable) } }
pub struct Api { binding_call: Option<unsafe extern "C" fn(*mut core::ffi::c_void,u32,*const u8,u32,*mut u8,u32)->i32> }
pub struct Context<'a> { raw:*mut core::ffi::c_void, api:&'a Api }
#[derive(Clone, Copy, Debug, Default, PartialEq)] pub struct ExternalRecord { pub x:f32, pub y:f32 }
pub mod enums { #[derive(Clone, Copy, Debug, Default, PartialEq)] pub struct ExternalEnum(pub i32); }
pub mod bindings { pub struct Sample<'c,'a> { pub(crate) context:&'c super::Context<'a> } }
pub mod imports {
    pub struct Import(u32); impl Import { pub fn slot(&self)->u32 {self.0} }
''' + '\n'.join(f'pub static BINDING_{i}:Import=Import({i});' for i in range(1, 14)) + '''
}
mod generated {
''' + generated + '''
}
struct Host { mode:u8, calls:usize, closes:usize }
fn record_bytes() -> Vec<u8> {
    let mut value=vec![(-2i8) as u8]; value.extend_from_slice(&(-123456i32).to_le_bytes());
    value.extend_from_slice(&0xfedcba9876543210u64.to_le_bytes()); value.push((-1i8) as u8);
    value.extend_from_slice(&0x7fc12345u32.to_le_bytes()); value.extend_from_slice(&1.25f32.to_le_bytes()); value
}
unsafe extern "C" fn call(raw:*mut core::ffi::c_void,id:u32,i:*const u8,ni:u32,o:*mut u8,no:u32)->i32 {
    let h=&mut *raw.cast::<Host>(); h.calls+=1;
    if h.mode==9 { return -3; }
    let input=if ni==0 {&[]} else {core::slice::from_raw_parts(i,ni as usize)};
    let output=if no==0 {&mut []} else {core::slice::from_raw_parts_mut(o,no as usize)};
    match id {
        1 | 2 => { assert_eq!((ni,no),(0,4));
            let token=if h.mode==0 {0u32} else if id==1 {0xfedcba98u32} else {0x89abcdefu32};
            output.copy_from_slice(&token.to_le_bytes()); }
        3 | 4 | 5 => {
            assert_eq!(ni,4); assert_eq!(no,if id==5 {0} else {4});
            let token=if id==4 {0x89abcdefu32} else {0xfedcba98u32}; assert_eq!(input,token.to_le_bytes());
            if id==5 { h.closes+=1; } else { output.copy_from_slice(&0x7fc12345u32.to_le_bytes()); }
        }
        6 | 13 => { assert_eq!((ni,no),(26,1)); let mut bytes=record_bytes();
            bytes.extend_from_slice(&(-9i32).to_le_bytes()); assert_eq!(input,bytes); output[0]=1; }
        7 | 8 => { assert_eq!((ni,no),(0,23)); output[0]=h.mode;
            if h.mode!=0 {output[1..].copy_from_slice(&record_bytes());} }
        9 | 10 => { assert_eq!((ni,no),(0,1)); output[0]=197; }
        11 => { assert_eq!((ni,no),(4,1)); assert_eq!(input,1.5f32.to_le_bytes()); output[0]=1; }
        12 => { assert_eq!((ni,no),(0,4)); output.copy_from_slice(&(-9i32).to_le_bytes()); }
        _ => panic!("unknown binding"),
    }
    0
}
#[test] fn exercise() {
    let mut host=Host{mode:1,calls:0,closes:0}; let api=Api{binding_call:Some(call)};
    let c=Context{raw:(&mut host as *mut Host).cast(),api:&api}; let s=bindings::Sample{context:&c};
    let owned=s.open().unwrap().unwrap(); let channel=s.channel().unwrap().unwrap();
    assert_eq!(owned.read(&c).unwrap().to_bits(),0x7fc12345);
    assert_eq!(channel.read(&c).unwrap().to_bits(),0x7fc12345);
    host.mode=9; assert_eq!(owned.read(&c),Err(Error::Unavailable));
    host.mode=1; assert_eq!(owned.read(&c).unwrap().to_bits(),0x7fc12345);
    owned.close(&c).unwrap(); assert_eq!(host.closes,1);
    let record=s.optional_a().unwrap().unwrap(); assert_eq!(record.small,-2); assert_eq!(record.number,-123456);
    assert_eq!(record.mask,0xfedcba9876543210); assert_eq!(record.tag,generated::Tag::Good);
    assert_eq!(record.vector.x.to_bits(),0x7fc12345); assert_eq!(record.vector.y,1.25);
    assert_eq!(s.write(&record,enums::ExternalEnum(-9)),Ok(true));
    assert_eq!(s.write_again(&record,enums::ExternalEnum(-9)),Ok(true));
    assert_eq!(s.optional_b().unwrap().unwrap().number,-123456);
    assert_eq!(s.named_a().unwrap().x,197); assert_eq!(s.named_b().unwrap().x,197);
    assert_eq!(s.bounded(1.5),Ok(true)); assert_eq!(s.rotation(),Ok(enums::ExternalEnum(-9)));
    let before=host.calls; assert_eq!(s.bounded(f32::NAN),Err(Error::Invalid));
    assert_eq!(s.bounded(0.5),Err(Error::Invalid)); assert_eq!(host.calls,before);
    host.mode=0; assert!(s.open().unwrap().is_none()); assert!(s.channel().unwrap().is_none());
    assert_eq!(s.optional_a(),Ok(None));
    host.mode=9; assert!(matches!(s.open(),Err(Error::Unavailable)));
    host.mode=1; let owned=s.open().unwrap().unwrap(); host.mode=9;
    assert_eq!(owned.close(&c),Err(Error::Unavailable)); assert_eq!(host.closes,1);
}
'''
    source.write_text(program)
    executable = tmp_path / 'owned'
    rustc = os.environ.get('RUSTC', 'rustc')
    subprocess.run([rustc, '--edition=2021', '--test', str(source), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
    source.write_text(program + '''
fn reuse_after_close(c:&Context<'_>, owned:generated::Owned) {
    let _=owned.close(c); let _=owned.read(c);
}
''')
    failed = subprocess.run([rustc, '--edition=2021', '--test', str(source), '-o', str(executable)],
                            capture_output=True, text=True)
    assert failed.returncode != 0 and 'E0382' in failed.stderr, failed.stderr

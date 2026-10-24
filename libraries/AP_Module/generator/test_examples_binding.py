# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for manual calls used by AP_Scripting examples."""

import subprocess

import bindings
import examples_binding


def test_examples_selection_and_emission():
    spec = examples_binding.select(bindings.read_ir(), 700)
    assert [method['name'] for method in spec['methods']] == [
        'vehicle.set_target_velocity_ned', 'srv_channels.get_safety_state',
        'mission.receive', 'DroneCan.get_flex_debug']
    assert [method['id'] for method in spec['methods']] == [700, 701, 702, 703]
    assert spec['methods'][2]['result']['fields'] == [
        ['time_ms', 'u32-le'], ['param1', 'u16-le'], ['param2', 'f32-le,all-bits'],
        ['param3', 'f32-le,all-bits'], ['param4', 'f32-le,all-bits']]
    assert spec['methods'][3]['result']['payload'] == 'bytes-max255-zero-pad255'
    cpp = '\n'.join(examples_binding.cpp(spec))
    assert 'BindingLock lock(AP::scheduler().get_semaphore())' in cpp
    assert 'safety_switch_state() != AP_HAL::Util::SAFETY_ARMED' in cpp
    assert '#if AP_MISSION_ENABLED && AP_MODULE_ENABLED' in cpp
    assert 'runtime->pop_mission_command(command)' in cpp
    assert 'AP::scripting()' not in cpp
    assert '#if HAL_ENABLE_DRONECAN_DRIVERS && AP_SCRIPTING_ENABLED' in cpp
    assert 'dronecan->get_FlexDebug(node_id, message_id, timestamp_us, message)' in cpp
    assert 'memcpy(output + 6, message.u8.data, message.u8.len)' in cpp
    rust = examples_binding.rust(spec, '')
    assert "impl crate::bindings::Vehicle<'_, '_>" in rust
    assert 'pub fn set_target_velocity_ned' in rust
    assert "impl crate::bindings::SrvChannels<'_, '_>" in rust
    assert 'pub fn get_safety_state' in rust
    assert 'pub struct MissionCommand' in rust
    assert "impl crate::bindings::Mission<'_, '_>" in rust
    assert 'pub fn receive(&self) -> Result<Option<MissionCommand>>' in rust
    assert 'pub struct DroneCanFlexDebug' in rust
    assert "impl crate::bindings::Dronecan<'_, '_>" in rust
    assert 'pub fn get_flex_debug' in rust
    assert 'output[6 + length..].iter().any' in rust


def test_examples_absence_is_zero_encoded_and_results_are_owned():
    spec = examples_binding.specification(1)
    cpp = '\n'.join(examples_binding.cpp(spec))
    rust = examples_binding.rust(spec, '')
    assert f'memset(output, 0, {examples_binding.MISSION_WIRE_SIZE});' in cpp
    assert f'memset(output, 0, {examples_binding.FLEXDEBUG_WIRE_SIZE});' in cpp
    assert 'complete-owned-copy' in spec['methods'][2]['policy']
    assert 'complete-owned-copy' in spec['methods'][3]['policy']
    assert rust.count("0 if output[1..].iter().all(|byte| *byte == 0) => Ok(None)") == 2
    assert 'payload: [u8; FLEXDEBUG_PAYLOAD_MAX]' in rust


def test_examples_emitted_sdk_compiles(tmp_path):
    generated = examples_binding.rust(examples_binding.specification(700), '')
    source = tmp_path / 'sdk.rs'
    imports = '\n'.join(f'pub(crate) static BINDING_{index}: Import = Import({index});'
                        for index in range(700, 704))
    source.write_text(r'''
use core::ffi::c_void;
#[derive(Clone, Copy, Debug, Eq, PartialEq)] pub enum Error { Host }
pub type Result<T=()> = core::result::Result<T, Error>;
fn result(status: i32) -> Result { if status == 0 { Ok(()) } else { Err(Error::Host) } }
pub struct Vector3f;
impl Vector3f { pub fn encode(&self) -> [u8; 12] { [0; 12] } }
struct HostApi {
    binding_call: Option<unsafe extern "C" fn(*mut c_void, u32, *const u8, u32, *mut u8, u32) -> i32>,
}
pub struct Context<'a> { api: &'a HostApi, raw: *mut c_void }
pub mod bindings {
    use super::Context;
    pub struct Vehicle<'context, 'api> { pub(crate) context: &'context Context<'api> }
    pub struct SrvChannels<'context, 'api> { pub(crate) context: &'context Context<'api> }
    pub struct Mission<'context, 'api> { pub(crate) context: &'context Context<'api> }
    pub struct Dronecan<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
impl<'api> Context<'api> {
    pub fn vehicle(&self) -> bindings::Vehicle<'_, 'api> { bindings::Vehicle { context: self } }
    pub fn srv_channels(&self) -> bindings::SrvChannels<'_, 'api> { bindings::SrvChannels { context: self } }
    pub fn mission(&self) -> bindings::Mission<'_, 'api> { bindings::Mission { context: self } }
    pub fn dronecan(&self) -> bindings::Dronecan<'_, 'api> { bindings::Dronecan { context: self } }
}
pub(crate) struct Import(u32);
impl Import { pub(crate) fn slot(&self) -> u32 { self.0 } }
pub(crate) mod imports { use super::Import;
''' + imports + '''
}
mod generated {
''' + generated + r'''
}
fn main() {}
''')
    subprocess.run(['rustc', '--edition=2021', str(source), '-o', str(tmp_path / 'sdk')], check=True)


def test_examples_coverage_and_dispatch_are_wired():
    spec = bindings.specification()
    coverage = {(entry['owner'], entry['name']): entry for entry in spec['coverage']['entries']}
    assert coverage[('AP_Vehicle', 'set_target_velocity_NED')]['status'] == 'generated'
    assert coverage[('SRV_Channels', 'get_safety_state')]['status'] == 'generated'
    source = bindings.emit(spec)['Bindings_generated.cpp']
    assert 'return binding_examples_vehicle_set_target_velocity_ned' in source
    assert 'return binding_examples_srv_channels_get_safety_state' in source

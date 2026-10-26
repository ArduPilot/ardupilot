# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lost-target handling must still run when no airspeed estimate is available."""

import os
import subprocess

from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]


def test_lost_target_precedes_missing_airspeed(tmp_path):
    source = (ROOT / 'libraries/AP_Module/applets/plane_follow/main.rs').read_text()
    start = source.index('        let current_altitude =')
    update = source[start:source.index('        self.lost_target_countdown =', start)]
    probe = tmp_path / 'follow.rs'
    probe.write_text('''
#![allow(unused_variables, unused_mut)]
#[derive(Debug, PartialEq)] enum Error { Unavailable }
type Result<T> = std::result::Result<T, Error>;
struct Location;
impl Location { fn alt(&self) -> i32 { 0 } }
struct Context { target: bool }
impl Context {
    fn vehicle(&self) -> &Self { self }
    fn follow(&self) -> &Self { self }
    fn ahrs(&self) -> &Self { self }
    fn get_target_location(&self) -> Result<Option<()>> { Ok(self.target.then_some(())) }
    fn get_target_location_and_velocity(&self) -> Result<Option<((),())>> {
        Ok(self.target.then_some(((),())))
    }
    fn get_target_location_and_velocity_ofs(&self) -> Result<Option<((),())>> {
        Ok(self.target.then_some(((),())))
    }
    fn get_target_heading_deg(&self) -> Result<Option<f32>> { Ok(None) }
    fn airspeed_eas(&self) -> Result<Option<f32>> { Ok(None) }
}
fn retry<T>(_: &Context, call: impl FnOnce() -> Result<T>) -> Result<T> { call() }
struct State { telemetry_failed: bool, losses: u32 }
impl State {
    fn handle_lost_target(&mut self, _: &Context) -> Result<()> { self.losses += 1; Ok(()) }
    fn update(&mut self, context: &Context) -> Result<()> {
        let current_location = Location;
''' + update + '''
        Ok(())
    }
}
fn main() {
    for (target, telemetry_failed, losses) in [(false,false,1), (true,true,1), (true,false,0)] {
        let mut state = State { telemetry_failed, losses: 0 };
        let result = state.update(&Context { target });
        assert_eq!(state.losses, losses);
        assert_eq!(result, if losses == 0 { Err(Error::Unavailable) } else { Ok(()) });
    }
}
''')
    binary = tmp_path / 'follow'
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', str(probe), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)

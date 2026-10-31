# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise parameter rejection, dynamic HTTP files and concurrent SYS snapshots."""

import http.client
import http.server
import os
import socket
import threading
import time

from pathlib import Path

import pytest

from pymavlink import mavutil
from test_runtime import ROOT
from test_runtime import Vehicle
from test_thread_info import FTPReader

import build


def test_onvif_serializes_lua_and_native(tmp_path, monkeypatch):
    binary = os.environ.get('AP_MODULE_ONVIF_BINARY')
    if binary is None:
        pytest.skip('Requires ONVIF-enabled Plane SITL')
    monkeypatch.setenv('AP_MODULE_SITL_BINARY', binary)
    seen = threading.Event()
    release = threading.Event()
    requests = []

    class Handler(http.server.BaseHTTPRequestHandler):
        def do_POST(self):
            self.rfile.read(int(self.headers.get('Content-Length', 0)))
            requests.append(self.path)
            seen.set()
            release.wait(20)
            self.send_response(500)
            self.send_header('Content-Length', '0')
            self.end_headers()

        def log_message(self, *_):
            pass

    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{server.server_port}'
    source = tmp_path / 'onvif.rs'
    source.write_text('''#![no_std]
#![no_main]
use ap_rust_sdk::{Context, Result};
ap_rust_sdk::module!(run);
fn run(context: &Context<'_>) -> Result {
    context.text(6, "Native ONVIF begin")?;
    context.onvif().start("user", "pass", "URL")?;
    context.text(6, "Native ONVIF finished")
}
'''.replace('URL', url))
    image = build.build(source, tmp_path / 'onvif.apmod', 'x86_64-unknown-linux-gnu', 16384)
    vehicle = Vehicle(tmp_path / 'vehicle', {}, {'SCR_ENABLE': 1, 'SCR_DIR_DISABLE': 1})
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    (scripts / 'onvif.lua').write_text('''local function update()
    if param:get("SCR_USER1") == 1 then
        onvif:start("user", "pass", "URL")
        gcs:send_text(6, "Lua ONVIF finished")
        return
    end
    return update, 50
end
return update, 50
'''.replace('URL', url))
    try:
        vehicle.start()
        vehicle.parameter('SCR_USER1', 1)
        assert seen.wait(5), vehicle.diagnostic()
        (vehicle.modules / 'onvif.apmod').write_bytes(image.read_bytes())
        vehicle.restart(1)
        vehicle.text('Native ONVIF begin')
        vehicle.pump(0.4)
        assert len(requests) == 1, requests
        release.set()
        vehicle.text('Lua ONVIF finished')
        vehicle.text('Native ONVIF finished')
        assert len(requests) == 2, requests
    finally:
        release.set()
        vehicle.stop()
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)


@pytest.mark.parametrize('first_owner', ['lua', 'native'])
def test_custom_mode_ownership_across_runtimes(tmp_path, monkeypatch, first_owner):
    binary = Path(os.environ.get('AP_MODULE_COPTER_BINARY', ROOT / 'build/sitl/bin/arducopter'))
    if not binary.is_file():
        pytest.skip('Requires Copter SITL with native modules and Lua')
    monkeypatch.setenv('AP_MODULE_SITL_BINARY', str(binary))
    source = tmp_path / 'mode.rs'
    source.write_text('''#![no_std]
#![no_main]
use ap_rust_sdk::{Context, Error, Result};
ap_rust_sdk::module!(run);
fn run(context: &Context<'_>) -> Result {
    let mode = context.vehicle().register_custom_mode(240, "CrossRuntime", "XRTM")?;
    if FIRST_LUA {
        if mode.is_some() { return Err(Error::Host); }
        context.text(6, "Native custom mode rejected")?;
        return Ok(());
    }
    let mode = mode.ok_or(Error::Host)?;
    mode.set_allow_entry(context, true)?;
    context.text(6, "Native custom mode held")?;
    while !context.stop_requested() {
        if !mode.allow_entry(context)? { return Err(Error::Host); }
        context.sleep_ms(10)?;
    }
    Ok(())
}
'''.replace('FIRST_LUA', str(first_owner == 'lua').lower()))
    image = build.build(source, tmp_path / 'mode.apmod', 'x86_64-unknown-linux-gnu', 8192)
    files = {} if first_owner == 'lua' else {'mode.apmod': image.read_bytes()}
    vehicle = Vehicle(tmp_path / 'vehicle', files, {
        'SCR_ENABLE': 1, 'SCR_DIR_DISABLE': 1, 'SCR_USER1': int(first_owner == 'lua'),
    }, model='+')
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    (scripts / 'mode.lua').write_text('''local requested = 0
local mode
local function update()
    local request = param:get("SCR_USER1")
    if request ~= requested and request > 0 then
        requested = request
        mode = vehicle:register_custom_mode(240, "CrossRuntime", "XRTM")
        if mode then
            mode:allow_entry(true)
            gcs:send_text(6, "Lua custom mode held")
        else
            gcs:send_text(6, "Lua custom mode rejected")
        end
    end
    if mode and not mode:allow_entry() then error("Lua custom mode lost") end
    return update, 50
end
return update, 50
''')
    try:
        vehicle.start()
        if first_owner == 'lua':
            vehicle.text('Lua custom mode held')
            (vehicle.modules / 'mode.apmod').write_bytes(image.read_bytes())
            vehicle.restart(1)
            vehicle.text('Native custom mode rejected')
        else:
            vehicle.text('Native custom mode held')
            vehicle.parameter('SCR_USER1', 1)
            vehicle.text('Lua custom mode rejected')
            vehicle.pump(0.3)
            (vehicle.modules / 'mode.apmod').unlink()
            vehicle.restart(1, expected=0)
            vehicle.parameter('SCR_USER1', 2)
            vehicle.text('Lua custom mode held')
        vehicle.pump(0.3)
        assert not any('returned -' in text or 'Lua custom mode lost' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


def test_lockdown_rejects_invalid_value_and_keeps_running(tmp_path):
    image = build.build(ROOT / 'libraries/AP_Module/applets/param_lockdown/main.rs',
                        tmp_path / 'lock.apmod', 'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {'lock.apmod': image.read_bytes()}, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.text('param-lockdown script loaded')
        vehicle.pump(0.2)
        vehicle.link.mav.param_set_send(1, 1, b'LOG_DISARMED', 200, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        reply = vehicle.until(lambda msg: msg.get_type() == 'PARAM_ERROR' and msg.param_id == 'LOG_DISARMED')
        assert reply.error == mavutil.mavlink.MAV_PARAM_ERROR_VALUE_OUT_OF_RANGE
        before = len(vehicle.texts)
        vehicle.link.mav.param_set_send(1, 1, b'LOG_DISARMED', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        vehicle.text('param set applied', after=before)
        assert vehicle.parameter('LOG_DISARMED') == 1
        before = len(vehicle.texts)
        vehicle.link.mav.param_set_send(1, 1, b'PARAM_LOCK_ENAB', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        vehicle.text('param set applied', after=before)
        vehicle.pump(0.2)
        assert vehicle.parameter('ARMING_REQUIRE', 0) == 0
        assert not any('returned -' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


def test_webserver_dynamic_sys_files(tmp_path):
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        port = available.getsockname()[1]
    image = build.build(ROOT / 'libraries/AP_Module/applets/net_webserver/main.rs',
                        tmp_path / 'web.apmod', 'x86_64-unknown-linux-gnu', 16384, heap_size=65536)
    vehicle = Vehicle(tmp_path / 'vehicle', {'web.apmod': image.read_bytes()},
                      {'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 65536})
    (vehicle.directory / 'example.txt').write_bytes(b'ordinary file\n')

    def get(path):
        deadline = time.monotonic() + 10
        while True:
            client = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
            try:
                client.request('GET', path)
                response = client.getresponse()
                return response.status, response.getheader('Content-Length'), response.read()
            except ConnectionRefusedError:
                if time.monotonic() >= deadline:
                    raise
                vehicle.pump(0.1)
            finally:
                client.close()

    try:
        vehicle.start()
        vehicle.text('WebServer: starting on port 8080')
        assert vehicle.parameter('WEB_BIND_PORT', port) == port
        vehicle.restart(1)
        vehicle.text(f'WebServer: starting on port {port}')
        status, _, page = get('/')
        assert status == 200 and b'update_status()' in page
        status, _, listing = get('/mnt/@SYS/')
        assert status == 200 and b'modules.txt' in listing
        status, length, contents = get('/mnt/@SYS/modules.txt')
        assert status == 200 and length is None
        assert contents.startswith(b'ModulesV1\n') and b'web.apmod' in contents
        status, length, contents = get('/mnt/example.txt')
        assert status == 200 and int(length) == len(contents)
        assert contents == b'ordinary file\n'
    finally:
        vehicle.stop()


def test_concurrent_native_and_ftp_sys_snapshots(tmp_path):
    source = tmp_path / 'sys.rs'
    source.write_text('''#![no_std]
#![no_main]
use ap_rust_sdk::{Context, Error, Result};
ap_rust_sdk::module!(run);
fn run(context: &Context<'_>) -> Result {
    let mut bytes = [0u8; 256];
    let mut ready = false;
    while !context.stop_requested() {
        let length = context.fs().read_at("@SYS/modules.txt", 0, &mut bytes)?.ok_or(Error::Host)?;
        if !bytes[..length].starts_with(b"ModulesV1\\n") { return Err(Error::Host); }
        if !ready { context.text(6, "SYS ready")?; ready = true; }
        context.sleep_ms(10)?;
    }
    Ok(())
}
''')
    image = build.build(source, tmp_path / 'sys.apmod', 'x86_64-unknown-linux-gnu', 8192).read_bytes()
    vehicle = Vehicle(tmp_path / 'vehicle', {f'sys{i}.apmod': image for i in range(3)}, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.until(lambda _: sum('SYS ready' in text for text in vehicle.texts) == 3)
        reader = FTPReader(vehicle)
        for _ in range(50):
            assert reader.read('@SYS/modules.txt').startswith('ModulesV1\n')
        assert not any('returned -' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()

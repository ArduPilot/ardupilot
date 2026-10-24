# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Fly the same RC-triggered AirShow in plane-3d, using either scripting engine."""

import shutil
import statistics
import time

import pytest

from pymavlink import DFReader
from pymavlink import mavutil
from test_aerobatics_path import APPLET
from test_aerobatics_path import LUA_DIR
from test_aerobatics_path import lua_airshow_schedule
from test_runtime import ROOT
from test_runtime import Vehicle

import build


class AerobaticVehicle(Vehicle):
    def __init__(self, directory, files, extra):
        super().__init__(directory, files, extra, model='plane-3d')
        base = ''.join((ROOT / 'Tools/autotest/models' / name).read_text() + '\n'
                       for name in ('plane.parm', 'plane-3d.parm'))
        self.defaults.write_text(base + self.defaults.read_text())
        self.rc_values = [1500, 1500, 1000, 1500, 1500, 1500, 1000, 1800, 1000] + [0] * 9
        self.last_rc = 0

    def start(self):
        super().start()
        self.link.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.pump(0.5)

    def receive(self):
        if time.monotonic() - self.last_rc > 0.1:
            self.link.mav.rc_channels_override_send(1, 1, *self.rc_values)
            self.last_rc = time.monotonic()
        return super().receive()

    def rc(self, channel, value):
        self.rc_values[channel-1] = value
        self.last_rc = 0
        self.pump(0.3)

    def mode(self, mode):
        number = self.link.mode_mapping()[mode]
        self.link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, number)
        self.until(lambda m: m.get_type() == 'HEARTBEAT' and m.custom_mode == number)


@pytest.mark.parametrize('engine', ['rust', 'lua'])
def test_airshow_flight(tmp_path, engine):
    assert mavutil.mavlink.WIRE_PROTOCOL_VERSION == '2.0', 'Run with MAVLINK20=1 for RC channel 9 overrides'
    files = None
    if engine == 'rust':
        module = build.build(APPLET / 'main.rs', tmp_path / 'aerobatics.apmod', 'x86_64-unknown-linux-gnu',
                             16384, heap_size=32768)
        assert b'AERO fault injected' not in module.read_bytes()
        assert b'AERO finish fault injected' not in module.read_bytes()
        files = {'airshow.apmod': module.read_bytes()}
    vehicle = AerobaticVehicle(tmp_path / 'vehicle', files, {
        'MOD_ENABLE': int(engine == 'rust'), 'SCR_ENABLE': int(engine == 'lua'),
        'MOD_HEAP_MAX': 32768, 'MOD_CODE_MAXSIZE': 131072,
        'MOD_STACK_MAX': 32768, 'SCR_DEBUG_OPTS': 72 if engine == 'lua' else 0,
        'LOG_BACKEND_TYPE': 1, 'LOG_DISARMED': 1, 'LOG_FILE_DSRMROT': 0,
        'LOG_FILE_BUFSIZE': 128, 'RC7_OPTION': 300, 'RC9_OPTION': 301,
        'RC_OVERRIDE_TIME': 3, 'FLTMODE_CH': 0,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    if engine == 'rust':
        shutil.copyfile(APPLET / 'AirShow.txt', scripts / 'trick72.txt')
    else:
        (scripts / 'trick72.txt').write_text(lua_airshow_schedule())
    if engine == 'lua':
        shutil.copyfile(LUA_DIR / 'plane_aerobatics.lua', scripts / 'plane_aerobatics.lua')
    try:
        vehicle.start()
        vehicle.parameter('TRIK_ENABLE', 1)
        if engine == 'rust':
            vehicle.restart(1)
            vehicle.text('Enabled 3 tricks')
        else:
            vehicle.link.mav.command_int_send(1, 1, 0, mavutil.mavlink.MAV_CMD_SCRIPTING, 0, 0,
                                              mavutil.mavlink.SCRIPTING_CMD_STOP_AND_RESTART, 0, 0, 0, 0, 0, 0)
            vehicle.text('Enabled 3 aerobatic tricks')
        vehicle.parameter('TRIK1_ID', 72)
        if engine == 'rust':
            # Reload with execute already selected must not launch a trick.
            vehicle.rc(7, 2000)
            mark = len(vehicle.texts)
            vehicle.restart(2)
            vehicle.pump(1)
            assert not any('Trick 1 started' in text for text in vehicle.texts[mark:]), vehicle.diagnostic()
            assert vehicle.parameter('TRIK1_ID') == 72
            vehicle.rc(7, 1000)
        vehicle.pump(3)
        vehicle.mode('TAKEOFF')
        vehicle.arm()
        vehicle.until(lambda m: m.get_type() == 'GLOBAL_POSITION_INT' and 30000 <= m.relative_alt <= 40000, 30)
        vehicle.mode('CRUISE')
        vehicle.rc(9, 1000)
        vehicle.rc(7, 1500)
        vehicle.text('Trick 1 selected (SuperAirShow)', timeout=20)
        worker_name = 'airshow.apmod' if engine == 'rust' else 'Scripting'
        worker_before = vehicle.worker(worker_name)
        assert worker_before is not None, worker_name
        flight_mark = len(vehicle.texts)
        vehicle.rc(7, 2000)
        vehicle.text('Trick 1 started (SuperAirShow)', timeout=20)
        peak = 0
        samples = 0
        deadline = time.monotonic() + 240
        while 'Finishing SuperAirShow!' not in vehicle.texts and time.monotonic() < deadline:
            m = vehicle.receive()
            if m is not None and m.get_type() == 'NAMED_VALUE_FLOAT' and m.name == 'PERR':
                peak = max(peak, m.value)
                samples += 1
                assert peak <= 15, (engine, peak, vehicle.diagnostic())
            assert not any('aborting' in text or 'panic' in text or 'Aerobatics error' in text
                           for text in vehicle.texts), vehicle.diagnostic()
        assert 'Finishing SuperAirShow!' in vehicle.texts, vehicle.diagnostic()
        worker_after = vehicle.worker(worker_name)
        assert worker_after is not None and worker_before[0] == worker_after[0]
        assert samples > 20 and peak > 0, (samples, peak)
        names = [line.split(':', 1)[1].strip() for line in (APPLET / 'AirShow.txt').read_text().splitlines()
                 if line.startswith('message:')]
        assert [text for text in vehicle.texts[flight_mark:] if text in names] == names, vehicle.diagnostic()
        print(f'{engine} full AirShow: {samples} PERR samples, peak {peak:.3f}m')
        print(f'{engine} worker CPU seconds: {worker_after[1]-worker_before[1]:.3f}')
        if engine == 'rust':
            # Reuse the retained schedule, verify messages restart, and abort.
            vehicle.rc(7, 1000)
            vehicle.rc(7, 1500)
            mark = len(vehicle.texts)
            vehicle.rc(7, 2000)
            vehicle.text('Trick 1 started', after=mark)
            vehicle.text('Loop', after=mark)
            vehicle.pump(1)
            vehicle.rc(7, 1000)
            vehicle.text('Trick aborted', after=mark)
            # Restart while controlling: the existing host watchdog releases
            # control, and the new worker must not act on the held switch.
            vehicle.rc(7, 1500)
            vehicle.rc(7, 2000)
            vehicle.pump(0.5)
            mark = len(vehicle.texts)
            vehicle.restart(3)
            vehicle.text('NavScript time out', after=mark)
            vehicle.pump(1)
            assert not any('Trick 1 started' in text for text in vehicle.texts[mark:]), vehicle.diagnostic()
            assert vehicle.parameter('TRIK1_ID') == 72
            print('Rust post-flight reload RAM:', [t for t in vehicle.texts[mark:] if t.startswith('Module RAM:')])
        vehicle.arm(False, force=True)
    finally:
        vehicle.stop()
    runtime = []
    memory = []
    started = False
    for file in sorted((vehicle.directory / 'logs').glob('*.BIN')):
        reader = DFReader.DFReader_binary(str(file))
        while (m := reader.recv_msg()) is not None:
            if m.get_type() == 'MSG':
                if m.Message.startswith('Trick 1 started'):
                    started = True
                elif m.Message == 'Finishing SuperAirShow!':
                    started = False
                    break
            if not started:
                continue
            if engine == 'rust' and m.get_type() == 'ARUN':
                runtime.append(m.Total)
            elif engine == 'lua' and m.get_type() == 'SCR':
                runtime.append(m.Runtime)
                memory.append(m.Total_mem)
    assert len(runtime) > 1000, len(runtime)
    print(f'{engine} update elapsed us: n={len(runtime)} mean={statistics.mean(runtime):.1f} '
          f'median={statistics.median(runtime):.1f} p99={sorted(runtime)[int(len(runtime)*0.99)]:.1f}')
    if memory:
        print(f'Lua maximum logged VM bytes (before GC): {max(memory)}')

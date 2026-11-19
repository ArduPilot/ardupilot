# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Deterministic Busy errors at the runner's read and command-delivery boundaries."""

import shutil

import pytest

from test_aerobatics_flight import AerobaticVehicle
from test_aerobatics_path import APPLET

import build


@pytest.mark.parametrize('fault', [1, 2, 3, 4, 5])
def test_runner_contention(tmp_path, fault):
    image = build.build(APPLET / 'main.rs', tmp_path / 'fault.apmod', 'x86_64-unknown-linux-gnu', 16384,
                        cfg=('ap_rust_aerobatics_faults',), heap_size=32768)
    assert b'AERO finish fault injected' in image.read_bytes()
    vehicle = AerobaticVehicle(tmp_path / 'vehicle', {'fault.apmod': image.read_bytes()}, {
        'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 32768, 'MOD_CODE_MAXSIZE': 131072, 'MOD_STACK_MAX': 32768,
        'RC7_OPTION': 300, 'RC9_OPTION': 301, 'RC_OVERRIDE_TIME': 3, 'FLTMODE_CH': 0,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    shutil.copyfile(APPLET / 'AirShow.txt', scripts / 'trick72.txt')
    try:
        vehicle.start()
        vehicle.parameter('TRIK_ENABLE', 1)
        vehicle.restart(1)
        vehicle.text('Enabled 3 tricks')
        vehicle.parameter('TRIK1_ID', 72)
        vehicle.pump(3)
        vehicle.mode('TAKEOFF')
        vehicle.arm()
        vehicle.until(lambda m: m.get_type() == 'GLOBAL_POSITION_INT' and 30000 <= m.relative_alt <= 40000, 30)
        vehicle.mode('CRUISE')
        vehicle.rc(7, 1500)
        vehicle.text('Trick 1 selected')
        vehicle.rc(7, 2000)
        vehicle.text('Trick 1 started')
        mark = len(vehicle.texts)
        if fault == 5:
            # Only the final delivery is blocked, independent of MAVLink latency.
            vehicle.parameter('TRIK1_ARG4', fault)
            vehicle.parameter('AEROM_ALT_ABORT', -10000)
            vehicle.text('Too low altitude, aborting', after=mark)
            vehicle.text('AERO finish fault injected', after=mark)
        else:
            vehicle.parameter('TRIK1_ARG4', fault)
            vehicle.text('AERO fault injected', after=mark)
        if fault in (3, 4):
            vehicle.text('Aerobatics: control timeout', after=mark)
        vehicle.pump(0.8)  # four simulated seconds, beyond all injection windows
        if fault in (1, 2):
            assert not any('timeout' in t or 'aborting' in t or 'Aerobatics error' in t
                           for t in vehicle.texts[mark:]), vehicle.diagnostic()
            vehicle.rc(7, 1000)
            vehicle.text('Trick aborted', after=mark)  # controller stayed active
        elif fault == 5:
            assert not any('timeout' in t for t in vehicle.texts[mark:]), vehicle.diagnostic()
            vehicle.rc(7, 1000)
            assert not any('Trick aborted' in t for t in vehicle.texts[mark:]), vehicle.diagnostic()
        # Every failure must leave the worker alive and allow a fresh request.
        vehicle.parameter('TRIK1_ARG4', 0)
        vehicle.parameter('AEROM_ALT_ABORT', 15)
        vehicle.rc(7, 1000)
        vehicle.rc(7, 1500)
        mark = len(vehicle.texts)
        vehicle.rc(7, 2000)
        vehicle.text('Trick 1 started', after=mark)
        vehicle.rc(7, 1000)
        vehicle.text('Trick aborted', after=mark)
        assert not any('panic' in t or 'returned -' in t for t in vehicle.texts), vehicle.diagnostic()
        vehicle.arm(False, force=True)
    finally:
        vehicle.stop()

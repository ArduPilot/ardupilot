# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lua-style armed-only heap expansion through the Rust worker and manager."""

import pytest

from test_runtime import ROOT
from test_runtime import Vehicle
from test_thread_info import FTPReader

import build


@pytest.mark.parametrize('disabled', [False, True])
def test_heap_expansion(tmp_path, disabled):
    module = build.build(ROOT / 'libraries/AP_Module/tests/modules/heap_expansion.rs',
                         tmp_path / 'expand.apmod', 'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {'expand.apmod': module.read_bytes()}, {
        'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 131072, 'MOD_DEBUG_OPTS': 64 if disabled else 0,
    })

    def request(size, expected):
        vehicle.parameter('REXP_REQUEST', size)
        vehicle.wait_parameter('REXP_RESULT', expected)
        listing = FTPReader(vehicle).read('@SYS/modules.txt')
        line = next(line for line in listing.splitlines() if line.startswith('expand.apmod '))
        fields = dict(field.split('=', 1) for field in line.split()[1:])
        free, capacity = map(int, fields['HEAP'].split('/'))
        assert 0 <= free <= capacity and 4096 <= capacity <= 131072, listing
        assert int(fields['ALLOC']) >= max(expected, 0), listing

    try:
        vehicle.start()
        vehicle.text('EXPAND ready')
        request(8192, -8192)
        assert not any('initial heap needs' in t for t in vehicle.texts)
        request(0, 0)
        vehicle.arm()
        request(8192, -8192 if disabled else 8192)
        if disabled:
            vehicle.pump(0.3)
            assert not any('initial heap needs' in t for t in vehicle.texts)
            vehicle.arm(False)
            vehicle.arm()  # a handled, non-expanding OOM does not latch an error
            vehicle.arm(False)
            # The option is sampled when modules are reloaded, just like Lua.
            vehicle.parameter('MOD_DEBUG_OPTS', 0)
            mark = len(vehicle.texts)
            vehicle.restart(1)
            vehicle.text('EXPAND ready', after=mark)
            vehicle.arm()
            request(8192, 8192)
        vehicle.text('initial heap needs')
        request(65536, 65536)  # a second expansion must raise the reported requirement
        vehicle.pump(0.3)
        warnings = [t for t in vehicle.texts if 'initial heap needs' in t]
        assert len(warnings) >= 2, vehicle.diagnostic()
        vehicle.arm(False)
        request(0, 0)
        request(8192, 8192)  # retained expanded memory is usable while disarmed
        request(262144, -262144)  # but no further disarmed expansion
        vehicle.arm(accepted=False)
        vehicle.text('rebuild with initial heap at least')
        vehicle.parameter('MOD_DEBUG_OPTS', 16)
        vehicle.arm()  # same pre-arm bypass as Lua
        vehicle.arm(False)
        vehicle.parameter('MOD_DEBUG_OPTS', 0)
        mark = len(vehicle.texts)
        vehicle.restart(2)
        vehicle.text('EXPAND ready', after=mark)
        vehicle.text('Module RAM: heap expanded to', after=mark)
        vehicle.arm()  # reload clears the expansion warning and frees the old heap
        vehicle.arm(False)
        request(8192, -8192)
        assert not any('panic' in t or 'returned -' in t for t in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


def test_expansion_survives_worker_exit(tmp_path):
    module = build.build(ROOT / 'libraries/AP_Module/tests/modules/heap_expansion.rs',
                         tmp_path / 'expand.apmod', 'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {'expand.apmod': module.read_bytes()},
                      {'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 131072})
    try:
        vehicle.start()
        vehicle.text('EXPAND ready')
        vehicle.arm()
        vehicle.parameter('REXP_REQUEST', -8192)
        vehicle.wait_parameter('REXP_RESULT', 8192)
        vehicle.text('Module: exited expand.apmod')
        vehicle.text('Module RAM: heap expanded to')
        vehicle.arm(False)
        vehicle.arm(accepted=False)
        vehicle.text('rebuild with initial heap at least')
    finally:
        vehicle.stop()

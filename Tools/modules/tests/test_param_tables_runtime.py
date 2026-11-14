# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Declarative tables through the real SDK, loader and AP_Param implementation."""

from test_runtime import ROOT, Vehicle

import build


def test_declarative_parameter_tables(tmp_path):
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/param_table.rs', tmp_path / 'table.apmod',
                        'x86_64-unknown-linux-gnu', 16384)
    vehicle = Vehicle(tmp_path / 'vehicle', {'table.apmod': image.read_bytes()}, {})
    try:
        vehicle.start()
        vehicle.text('TABLE ready')
        assert vehicle.parameter('RTBL_LAST') == 63
        assert vehicle.parameter('RTC_ENABLE') == 1
        vehicle.parameter('RTBL_SAVED', 42)
        vehicle.text('TABLE updated')
        assert vehicle.parameter('RTBL_SAVED') == 43
        assert vehicle.parameter('RTC_HIDDEN') == 19
        vehicle.pump(0.2)
        mark = len(vehicle.texts)
        vehicle.restart(1)
        vehicle.text('TABLE ready', after=mark)
        assert vehicle.parameter('RTBL_SAVED') == 43
    finally:
        vehicle.stop()

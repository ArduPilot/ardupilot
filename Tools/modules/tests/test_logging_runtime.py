# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Read actual DataFlash records written by Rust and Lua using shared formats."""

import math
import os

from collections import defaultdict

import pytest

from pymavlink import DFReader
from test_runtime import ROOT
from test_runtime import Vehicle

import build


@pytest.mark.parametrize('lua', [False, True])
def test_dynamic_logging(tmp_path, lua):
    if lua and os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Explicit Rust-only build')
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/logs.rs', tmp_path / 'logs.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    settings = {'LOG_BACKEND_TYPE': 1, 'LOG_DISARMED': 1, 'LOG_FILE_DSRMROT': 0}
    if lua:
        settings.update(SCR_ENABLE=1, SCR_HEAP_SIZE=100000)
    vehicle = Vehicle(tmp_path / 'vehicle', {'logs.apmod': image.read_bytes()}, settings)
    if lua:
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        (scripts / 'logger.lua').write_text('''local function update()
  logger.write("RLGP", "Height,Lat,Who", "fLB", "mD#", "0G-", 200.5, -353632610, 2)
  gcs:send_text(6, "LUA LOG wrote")
  return update, 100
end
return update()
''')
    try:
        vehicle.start()
        for cycle in range(3):
            start = len(vehicle.texts) if cycle else 0
            if cycle:
                vehicle.restart(cycle)
            vehicle.text('Module: MATH ok', after=start)
            if lua:
                vehicle.text('LUA LOG wrote', after=start)
            vehicle.pump(1)  # allow the file backend to flush
        assert not any('returned -' in line or 'panic' in line for line in vehicle.texts)
    finally:
        vehicle.stop()

    files = sorted((vehicle.directory / 'logs').glob('*.BIN'))
    assert files, vehicle.diagnostic()
    messages = defaultdict(list)
    formats = {}
    format_units = {}
    for path in files:
        reader = DFReader.DFReader_binary(str(path), zero_time_base=True)
        while (message := reader.recv_msg()) is not None:
            kind = message.get_type()
            if kind == 'FMT' and message.Name in ('RLG1', 'RLG2', 'RLGP'):
                entry = (message.Type, message.Format, message.Columns)
                if message.Name in formats:
                    assert formats[message.Name] == entry  # same registration after reload
                formats[message.Name] = entry
            elif kind == 'FMTU':
                format_units[message.FmtType] = (message.UnitIds, message.MultIds)
            elif kind in ('RLG1', 'RLG2', 'RLGP'):
                messages[kind].append(message)
    assert set(formats) == {'RLG1', 'RLG2', 'RLGP'}
    assert formats['RLG1'][1] == 'QbBhHiIQfcCeELMn'
    assert formats['RLG2'][1] == 'QNZff'
    assert formats['RLGP'][1:] == ('QfLB', 'TimeUS,Height,Lat,Who')
    for kind in ('RLG1', 'RLG2', 'RLGP'):
        assert len(messages[kind]) >= 100
        times = [m.TimeUS for m in messages[kind]]
        assert times == sorted(times) and times[0] > 0
    for msg in messages['RLG1']:
        assert (msg.A, msg.B, msg.C, msg.D, msg.E, msg.F, msg.G, msg.H) == (
            -8, 250, -1234, 65000, -1234567, 4000000000, 0xfedcba9876543210, 1.25)
        # DFReader applies format scaling for c/C/e/E/L; raw wire is unit-tested.
        assert (msg.I, msg.J, msg.K, msg.L, msg.M, msg.N, msg.O) == (
            -1, 1, -1000, 1000, -35.363261, 3, 'AB')
    for msg in messages['RLG2']:
        assert (msg.Short, msg.Long) == ('sixteen', 'sixty-four')
        assert math.isnan(msg.Nan) and msg.Inf == math.inf
    assert {m.Who for m in messages['RLGP']} == ({1, 2} if lua else {1})
    for msg in messages['RLGP']:
        assert msg.Height == (100.5 if msg.Who == 1 else 200.5)
        assert msg.Lat == -35.363261
    units, multipliers = format_units[formats['RLGP'][0]]
    assert units == 'smD#' and multipliers == 'F0G-'

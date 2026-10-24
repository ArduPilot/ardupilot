# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise synthetic native varargs through the real firmware formatter."""

import re

import pytest

from test_runtime import ROOT
from test_runtime import Vehicle

import build


@pytest.mark.parametrize('name', ['printf', 'simple_loop'])
def test_printf_sitl(tmp_path, name):
    source = ROOT / ('libraries/AP_Module/tests/modules/printf.rs' if name == 'printf'
                     else 'libraries/AP_Module/examples/simple_loop.rs')
    image = build.build(source, tmp_path / f'{name}.apmod', 'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {})
    try:
        vehicle.start()
        if name == 'printf':
            vehicle.text('Module: MATH ok')
        else:
            message = vehicle.until(lambda msg: msg.get_type() == 'STATUSTEXT' and
                                    re.fullmatch(r'Tick [0-9]+\.[0-9]', msg.text))
            assert message.severity == 6
            first = float(message.text[5:])
            message = vehicle.until(lambda msg: msg.get_type() == 'STATUSTEXT' and
                                    re.fullmatch(r'Tick [0-9]+\.[0-9]', msg.text))
            assert 0.9 <= float(message.text[5:]) - first <= 1.1
    finally:
        vehicle.stop()


@pytest.mark.parametrize('mode,diagnostic', [
    ('literal', 'Module fatal: P at '),
    ('display', 'Module fatal: Rust panic (formatted message'),
    ('stopped', 'Module fatal: stopped'),
])
def test_panic_printf_sitl(tmp_path, monkeypatch, mode, diagnostic):
    monkeypatch.setenv('AP_MODULE_PANIC_TEST', mode)
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/panic.rs', tmp_path / 'panic.apmod',
                        'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {})
    try:
        vehicle.start()
        if mode == 'stopped':
            vehicle.text('panic test ready')
            vehicle.parameter('MOD_RESTART', 1)
        vehicle.text('panic; reboot required')
        vehicle.text(diagnostic)
        assert not any('user Display must not run' in text for text in vehicle.texts)
    finally:
        vehicle.stop()

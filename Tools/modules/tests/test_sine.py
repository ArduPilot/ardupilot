# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Graph-demo messages from Lua, Rust, Ada and Fortran sharing one parameter."""

import math
import os
import shutil
import statistics
import time

import pytest

from test_runtime import ROOT
from test_runtime import Vehicle

import build

EXAMPLE = ROOT / 'libraries/AP_Module/examples/sine'
PHASES = dict(Sine_lua=0.0, Sine_rust=math.pi / 2, Sine_ada=math.pi, Sine_f90=3 * math.pi / 2)


@pytest.fixture(scope='module')
def sine_images(tmp_path_factory):
    if os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Requires Lua alongside native modules')
    for variable, default in [('ADA', 'llvm-gnat1'), ('FC', 'gfortran')]:
        if shutil.which(os.environ.get(variable, default)) is None:
            pytest.skip('Requires GNAT LLVM and GNU Fortran')
    directory = tmp_path_factory.mktemp('sine-images')
    images = {}
    for language, extension in [('rust', 'rs'), ('ada', 'adb'), ('f90', 'f90')]:
        output = directory / f'sine_{language}.apmod'
        build.build(EXAMPLE / language / f'sine.{extension}', output,
                    'x86_64-unknown-linux-gnu', 8192, heap_size=4096)
        images[output.name] = output.read_bytes()
    return images


def collect(vehicle, count):
    rows = {name: [] for name in PHASES}
    deadline = time.monotonic() + 20
    while min(len(samples) for samples in rows.values()) < count and time.monotonic() < deadline:
        message = vehicle.receive()
        if message is not None and message.get_type() == 'NAMED_VALUE_FLOAT' and message.name in rows:
            rows[message.name].append((message.time_boot_ms, message.value))
    assert all(len(samples) >= count for samples in rows.values()), (
        {name: len(samples) for name, samples in rows.items()}, vehicle.diagnostic())
    return rows


def check_samples(rows, frequency):
    for name, samples in rows.items():
        intervals = [second[0] - first[0] for first, second in zip(samples, samples[1:])]
        assert 80 <= statistics.median(intervals) <= 130, (name, intervals)
        for milliseconds, value in samples:
            expected = math.sin(2 * math.pi * frequency * milliseconds * 0.001 + PHASES[name])
            assert math.isclose(value, expected, abs_tol=0.03), (name, milliseconds, value, expected)


@pytest.mark.parametrize('first', ['lua', 'native'])
def test_four_languages_share_frequency(tmp_path, sine_images, first):
    vehicle = Vehicle(tmp_path / 'vehicle', {} if first == 'lua' else sine_images, {
        'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 128000, 'MOD_STACK_MAX': 32768,
        'MOD_CODE_MAXSIZE': 131072,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    lua = (EXAMPLE / 'lua/sine.lua').read_text()
    if first == 'native':
        head, tail = lua.rsplit('return initialise, 100', 1)
        lua = head + 'return initialise, 5000' + tail
    (scripts / 'sine.lua').write_text(lua)
    try:
        vehicle.start()
        vehicle.link.mav.param_request_read_send(1, 1, b'SINE_FREQ', -1)
        value = vehicle.until(lambda m: m.get_type() == 'PARAM_VALUE' and m.param_id == 'SINE_FREQ').param_value
        assert value == pytest.approx(0.2)
        if first == 'lua':
            for name, data in sine_images.items():
                (vehicle.modules / name).write_bytes(data)
            vehicle.restart(1, expected=3)
        workers = {name: vehicle.worker(name) for name in sine_images}
        assert all(worker is not None for worker in workers.values()), (workers, vehicle.diagnostic())
        for frequency in (0.2, 0.4, 0.0):
            vehicle.parameter('SINE_FREQ', frequency)
            vehicle.pump(0.5)
            check_samples(collect(vehicle, 40), frequency)
            assert all(vehicle.worker(name)[0] == worker[0] for name, worker in workers.items())
        vehicle.restart(2, expected=3)
        assert vehicle.parameter('SINE_FREQ') == 0
        vehicle.pump(0.5)
        check_samples(collect(vehicle, 20), 0.0)
        assert not any('panic' in text or 'returned -' in text or 'Lua: Error' in text
                       for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()

# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compile the actual Ada port and compare its replay with the Rust controller."""

import math
import os
import shutil
import struct
import subprocess

from pathlib import Path

import pytest

from build_airshow import build as build_ada
from pymavlink import DFReader
from test_aerobatics_flight import AerobaticVehicle
from test_runtime import ROOT
from test_runtime import Vehicle

import build

ADA = Path(__file__).resolve().parents[1]
APPLET = ADA.parent
NATIVE = 'x86_64-unknown-linux-gnu'
ARM = 'thumbv7em-none-eabihf'


@pytest.fixture
def gnat(monkeypatch):
    if shutil.which(os.environ.get('ADA', 'llvm-gnat1')) is None:
        pytest.skip('Requires GNAT LLVM; set ADA to llvm-gnat1')

    def select(target):
        name = 'ADA_ARM_INCLUDE_PATH' if target == ARM else 'ADA_NATIVE_INCLUDE_PATH'
        if name in os.environ:
            monkeypatch.setenv('ADA_INCLUDE_PATH', os.environ[name])
    return select


@pytest.mark.parametrize('target,cpu', [(NATIVE, 'x86-64'), (ARM, 'cortex-m4'), (ARM, 'cortex-m7')])
def test_build(tmp_path, gnat, target, cpu):
    gnat(target)
    image = build_ada(ADA / 'aerobatics.adb', tmp_path / 'airshow.apmod', target, 16384,
                      heap_size=4096, cpu=cpu)
    subprocess.run(['python3', str(ROOT / 'Tools/modules/apmod_info.py'), str(image)], check=True)


@pytest.mark.parametrize('variant', ['nominal', 'sweep', 'mirror'])
def test_controller_replay(tmp_path, monkeypatch, gnat, variant):
    gnat(NATIVE)
    text = (ADA / 'aerobatics.adb').read_text()
    marker = '   function Run (Ctx : AP_Module.Context) return AP_Module.Status is\n'
    assert text.count(marker) == 1
    replay = (ADA / 'tests/replay.adb.inc').read_text()
    replay = replay.replace('TEST_VARIANT', str(['nominal', 'sweep', 'mirror'].index(variant)))
    source = tmp_path / 'aerobatics.adb'
    source.write_text(text.split(marker)[0] + replay)
    shutil.copyfile(ADA / 'aerobatics.ads', source.with_suffix('.ads'))
    ada = build_ada(source, tmp_path / 'ada.apmod', NATIVE, 16384, heap_size=4096)
    monkeypatch.delenv('AP_MODULE_AERO_CONTROLLER_VARIANT', raising=False)
    if variant != 'nominal':
        monkeypatch.setenv('AP_MODULE_AERO_CONTROLLER_VARIANT', variant)
    rust = build.build(ROOT / 'libraries/AP_Module/tests/modules/aerobatics_controller.rs',
                       tmp_path / 'rust.apmod', NATIVE, 16384, heap_size=32768)
    vehicle = Vehicle(tmp_path / 'vehicle', {'ada.apmod': ada.read_bytes(), 'rust.apmod': rust.read_bytes()}, {
        'MOD_HEAP_MAX': 65536, 'MOD_CODE_MAXSIZE': 262144,
        'LOG_BACKEND_TYPE': 1, 'LOG_DISARMED': 1, 'LOG_FILE_DSRMROT': 0, 'LOG_FILE_BUFSIZE': 128,
        'AIRSPEED_MIN': 9, 'AIRSPEED_CRUISE': 18, 'SCALING_SPEED': 15,
    })
    shutil.copyfile(APPLET / 'AirShow.txt', vehicle.directory / 'trick72.txt')
    try:
        vehicle.start()
        vehicle.text('AEROP Ada controller done', timeout=45)
        vehicle.text('AEROP controller done', timeout=45)
        vehicle.pump(2)
        assert not any('panic' in text or 'returned -' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()
    rows = {name: {} for name in ('ADCT', 'ARCT', 'ADCS', 'ARCS', 'ADSP', 'RASP', 'ADHS', 'ARHS')}
    for file in sorted((vehicle.directory / 'logs').glob('*.BIN')):
        reader = DFReader.DFReader_binary(str(file))
        while (message := reader.recv_msg()) is not None:
            if message.get_type() in rows:
                rows[message.get_type()][int(message.Id)] = message.to_dict()
    maximum = 0.0
    for ada_name, rust_name in [('ADCT', 'ARCT'), ('ADCS', 'ARCS'), ('ADSP', 'RASP'), ('ADHS', 'ARHS')]:
        assert len(rows[ada_name]) == len(rows[rust_name]) == 512
        for index, actual in rows[ada_name].items():
            expected = rows[rust_name][index]
            for field, value in actual.items():
                if field in ('mavpackettype', 'TimeUS'):
                    continue
                reference = expected[field]
                assert math.isfinite(value) and math.isfinite(reference), (variant, index, field)
                maximum = max(maximum, abs(value - reference))
                assert struct.pack('<f', value) == struct.pack('<f', reference), (
                    variant, ada_name, index, field, value, reference)
    print(variant, 'maximum absolute difference', maximum)


def test_runner_parameters_and_reload(tmp_path, gnat):
    gnat(NATIVE)
    image = build_ada(ADA / 'aerobatics.adb', tmp_path / 'airshow.apmod', NATIVE, 16384, heap_size=4096)
    vehicle = AerobaticVehicle(tmp_path / 'vehicle', {'airshow.apmod': image.read_bytes()}, {
        'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 4096, 'MOD_CODE_MAXSIZE': 262144,
        'MOD_STACK_MAX': 32768, 'RC7_OPTION': 300, 'RC9_OPTION': 301,
        'RC_OVERRIDE_TIME': 3, 'FLTMODE_CH': 0,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    shutil.copyfile(APPLET / 'AirShow.txt', scripts / 'trick72.txt')
    try:
        vehicle.start()
        vehicle.text('Loaded plane_aerobatics Ada')
        vehicle.parameter('AEROM_LANG', 1)
        assert vehicle.parameter('TRIK_ENABLE') == 0
        assert vehicle.parameter('AEROM_ANG_ACCEL') == 6000
        vehicle.parameter('TRIK_ENABLE', 1)
        vehicle.restart(1)
        vehicle.text('Enabled 3 tricks')
        assert vehicle.parameter('TRIK_COUNT') == 3
        vehicle.parameter('TRIK_COUNT', 11)
        vehicle.restart(2)
        vehicle.text('Enabled 11 tricks')
        assert vehicle.parameter('TRIK11_ARG4') == 0
        vehicle.parameter('AEROM_ANG_ACCEL', 50)
        vehicle.parameter('AEROM_ANG_TC', 2)
        vehicle.parameter('TRIK1_ID', 72)
        vehicle.restart(3)
        assert vehicle.parameter('AEROM_ANG_ACCEL') == 3000
        assert vehicle.parameter('TRIK1_ID') == 72
        vehicle.rc(7, 2000)
        mark = len(vehicle.texts)
        vehicle.restart(4)
        vehicle.pump(1)
        assert not any('Trick 1 started' in text for text in vehicle.texts[mark:]), vehicle.diagnostic()
        vehicle.rc(7, 1000)
        vehicle.mode('CRUISE')
        vehicle.rc(7, 1500)
        vehicle.text('Trick 1 selected (SuperAirShow)')
        mark = len(vehicle.texts)
        vehicle.rc(7, 2000)
        vehicle.text('Trick 1 started (SuperAirShow)', after=mark)
        vehicle.rc(7, 1000)
        vehicle.text('Trick aborted', after=mark)
        assert not any('panic' in text or 'returned -' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


def test_full_airshow_flight(tmp_path, monkeypatch, gnat):
    from types import SimpleNamespace

    import test_aerobatics_flight as flight

    gnat(NATIVE)
    original_vehicle = flight.AerobaticVehicle

    class AdaFlightVehicle(original_vehicle):
        def __init__(self, directory, files, extra):
            super().__init__(directory, files, {**extra, 'MOD_CODE_MAXSIZE': 262144, 'MOD_HEAP_MAX': 4096})

        def start(self):
            super().start()
            self.parameter('AEROM_LANG', 1)

    def ada_image(unused_source, output, target, stack_size, **unused_options):
        return build_ada(ADA / 'aerobatics.adb', output, target, stack_size, heap_size=4096)

    monkeypatch.setattr(flight, 'build', SimpleNamespace(build=ada_image))
    monkeypatch.setattr(flight, 'AerobaticVehicle', AdaFlightVehicle)
    flight.test_airshow_flight(tmp_path, 'rust')

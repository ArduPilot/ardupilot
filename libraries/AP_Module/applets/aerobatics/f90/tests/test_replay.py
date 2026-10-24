# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compare the Fortran translation with the original Rust using real SITL bindings."""

import importlib.util
import math
import shutil
import struct
import sys

from pathlib import Path

import pytest

from pymavlink import DFReader

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
sys.path.insert(0, str(ROOT / 'Tools/modules'))
sys.path.insert(0, str(ROOT / 'Tools/modules/tests'))

from test_runtime import Vehicle  # noqa: E402

spec = importlib.util.spec_from_file_location('aerobatics_fortran_build', HERE.parent / 'build.py')
fortran = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fortran)


@pytest.mark.parametrize('variant', ['nominal', 'sweep', 'mirror'])
def test_controller_matches_rust(tmp_path, monkeypatch, variant):
    monkeypatch.setenv('MAVLINK20', '1')
    monkeypatch.delenv('AP_MODULE_AERO_CONTROLLER_VARIANT', raising=False)
    if variant != 'nominal':
        monkeypatch.setenv('AP_MODULE_AERO_CONTROLLER_VARIANT', variant)
    entry = tmp_path / 'replay.f90'
    variant_id = dict(nominal=0, sweep=1, mirror=2)[variant]
    entry.write_text((HERE / 'replay.f90').read_text().replace('variant=0', f'variant={variant_id}'))
    native = fortran.build(tmp_path / 'fortran.apmod', 'x86_64-unknown-linux-gnu', entry=entry)
    rust = fortran.module_build.build(ROOT / 'libraries/AP_Module/tests/modules/aerobatics_controller.rs',
                                      tmp_path / 'rust.apmod', 'x86_64-unknown-linux-gnu', 16384, heap_size=32768)
    vehicle = Vehicle(tmp_path / 'vehicle', {'fortran.apmod': native.read_bytes(), 'rust.apmod': rust.read_bytes()}, {
        'MOD_HEAP_MAX': 131072, 'MOD_CODE_MAXSIZE': 131072, 'MOD_STACK_MAX': 65536,
        'LOG_BACKEND_TYPE': 1, 'LOG_DISARMED': 1, 'LOG_FILE_DSRMROT': 0, 'LOG_FILE_BUFSIZE': 128,
        'AIRSPEED_MIN': 9, 'AIRSPEED_CRUISE': 18, 'SCALING_SPEED': 15,
    })
    shutil.copyfile(HERE.parents[1] / 'AirShow.txt', vehicle.directory / 'trick72.txt')
    try:
        vehicle.start()
        vehicle.text('AEROP Fortran controller done', timeout=60)
        vehicle.text('AEROP controller done', timeout=60)
        vehicle.pump(2)
        assert not any('panic' in line or 'returned -' in line for line in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()
    names = ('AFCT', 'ARCT', 'AFCS', 'ARCS', 'FASP', 'RASP', 'AFHS', 'ARHS')
    rows = {name: {} for name in names}
    for file in sorted((vehicle.directory / 'logs').glob('*.BIN')):
        reader = DFReader.DFReader_binary(str(file))
        while (record := reader.recv_msg()) is not None:
            if record.get_type() in rows:
                rows[record.get_type()][record.Id] = record.to_dict()
    maximum = 0.0
    for native_name, rust_name in [('AFCT', 'ARCT'), ('AFCS', 'ARCS'), ('FASP', 'RASP'), ('AFHS', 'ARHS')]:
        assert len(rows[native_name]) == len(rows[rust_name]) == 512
        for index, record in rows[rust_name].items():
            for field, expected in record.items():
                if field in ('mavpackettype', 'TimeUS'):
                    continue
                actual = rows[native_name][index][field]
                assert math.isfinite(actual) and math.isfinite(expected)
                maximum = max(maximum, abs(actual - expected))
                assert struct.pack('<f', actual) == struct.pack('<f', expected), (
                    variant, native_name, index, field, actual, expected)
    print(variant, '512 observations: maximum absolute difference', maximum)


def test_airshow_runner(tmp_path, monkeypatch):
    """Reuse the existing RC flight/reload/abort acceptance scenario for this port."""
    from types import SimpleNamespace

    sys.path.insert(0, str(ROOT / 'Tools/modules/rust/tests'))
    import test_aerobatics_flight as flight

    original_vehicle = flight.AerobaticVehicle

    class FortranFlightVehicle(original_vehicle):
        def start(self):
            super().start()
            self.parameter('AEROM_LANG', 2)

    def build_fortran(_source, output, target, stack_size, heap_size):
        return fortran.build(output, target, stack_size=stack_size, heap_size=heap_size)

    monkeypatch.setattr(flight, 'build', SimpleNamespace(build=build_fortran))
    monkeypatch.setattr(flight, 'AerobaticVehicle', FortranFlightVehicle)
    flight.test_airshow_flight(tmp_path, 'rust')

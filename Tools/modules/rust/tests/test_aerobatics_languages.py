# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Select live aerobatics workers individually and with all three loaded."""

import importlib.util
import os
import shutil
import time

import pytest

from pymavlink import DFReader
from pymavlink import mavutil
from test_aerobatics_flight import AerobaticVehicle
from test_aerobatics_path import APPLET

import build


def applet_builder(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build


@pytest.fixture(scope='module')
def language_images(tmp_path_factory):
    if shutil.which(os.environ.get('ADA', 'llvm-gnat1')) is None or \
            shutil.which(os.environ.get('FC', 'gfortran')) is None:
        pytest.skip('Requires GNAT LLVM and GNU Fortran')
    directory = tmp_path_factory.mktemp('aerobatics-languages')
    ada = applet_builder('aerobatics_ada_builder', APPLET / 'ada/build_airshow.py')
    fortran = applet_builder('aerobatics_f90_builder', APPLET / 'f90/build.py')
    images = {}
    target = 'x86_64-unknown-linux-gnu'
    images[0] = build.build(APPLET / 'main.rs', directory / 'rust.apmod', target, 16384, heap_size=32768)
    previous = os.environ.get('ADA_INCLUDE_PATH')
    try:
        if 'ADA_NATIVE_INCLUDE_PATH' in os.environ:
            os.environ['ADA_INCLUDE_PATH'] = os.environ['ADA_NATIVE_INCLUDE_PATH']
        images[1] = ada(APPLET / 'ada/aerobatics.adb', directory / 'ada.apmod', target, 16384, heap_size=4096)
    finally:
        if previous is None:
            os.environ.pop('ADA_INCLUDE_PATH', None)
        else:
            os.environ['ADA_INCLUDE_PATH'] = previous
    images[2] = fortran(directory / 'f90.apmod', target)
    return images


@pytest.mark.parametrize('loaded', [(0,), (1,), (2,), (0, 1, 2)])
def test_language_selection(tmp_path, language_images, loaded):
    files = {language_images[i].name: language_images[i].read_bytes() for i in loaded}
    vehicle = AerobaticVehicle(tmp_path / 'vehicle', files, {
        'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 32768, 'MOD_CODE_MAXSIZE': 262144,
        'MOD_STACK_MAX': 32768, 'RC7_OPTION': 300, 'RC9_OPTION': 301,
        'RC_OVERRIDE_TIME': 3, 'FLTMODE_CH': 0,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    shutil.copyfile(APPLET / 'AirShow.txt', scripts / 'trick72.txt')
    try:
        vehicle.start()
        assert vehicle.parameter('AEROM_LANG') == 0
        vehicle.parameter('TRIK_ENABLE', 1)
        restart_mark = len(vehicle.texts)
        vehicle.restart(1, expected=len(files))
        vehicle.parameter('TRIK1_ID', 72)
        vehicle.mode('CRUISE')
        workers = {name: vehicle.worker(name) for name in files}
        assert all(worker is not None for worker in workers.values())
        if 0 in loaded:
            vehicle.text('Aerobatics RUST selected', after=restart_mark)
        previous_language = 0
        names = {0: 'RUST', 1: 'ADA', 2: 'F90'}
        for language in (3, 0, 0, 1, 1, 2, 2, -1, 2, 1, 0):
            language_mark = len(vehicle.texts)
            assert vehicle.parameter('AEROM_LANG', language) == language
            vehicle.rc(7, 1000)
            vehicle.pump(0.5)
            mark = len(vehicle.texts)
            vehicle.rc(7, 1500)
            if language in loaded:
                vehicle.text('Trick 1 selected (SuperAirShow)', after=mark)
            vehicle.pump(1)
            selections = [text for text in vehicle.texts[mark:] if 'Trick 1 selected (SuperAirShow)' in text]
            assert len(selections) == int(language in loaded), (language, loaded, vehicle.diagnostic())
            announcements = [text for text in vehicle.texts[language_mark:]
                             if text.startswith('Aerobatics ') and text.endswith(' selected')]
            expected = []
            if language in loaded and language != previous_language:
                expected = [f'Aerobatics {names[language]} selected']
            assert announcements == expected, (language, loaded, announcements, vehicle.diagnostic())
            previous_language = language
            assert all(vehicle.worker(name)[0] == worker[0] for name, worker in workers.items())
        if len(loaded) == 3:
            # Exercise selector changes faster than one worker's update period.
            for i in range(90):
                vehicle.link.mav.param_set_send(1, 1, b'AEROM_LANG', i % 3,
                                                mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                vehicle.pump(0.003)
            vehicle.parameter('AEROM_LANG', 3)
            vehicle.pump(0.2)
            mark = len(vehicle.texts)
            vehicle.parameter('AEROM_LANG', 2)
            vehicle.text('Aerobatics F90 selected', after=mark)
        assert not any('panic' in text or 'returned -' in text or 'Aerobatics error' in text
                       for text in vehicle.texts), vehicle.diagnostic()
        vehicle.restart(2, expected=len(files))
        assert vehicle.parameter('AEROM_LANG') == (2 if len(loaded) == 3 else 0)
    finally:
        vehicle.stop()


def idle_language_vehicle(tmp_path, language_images, language):
    image = language_images[language]
    vehicle = AerobaticVehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {
        'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 32768, 'MOD_CODE_MAXSIZE': 262144,
        'MOD_STACK_MAX': 32768, 'RC7_OPTION': 300, 'RC9_OPTION': 301,
        'RC_OVERRIDE_TIME': 3, 'FLTMODE_CH': 0,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    shutil.copyfile(APPLET / 'AirShow.txt', scripts / 'trick72.txt')
    return vehicle


@pytest.mark.parametrize('language', [0, 1, 2])
def test_idle_language_requires_fresh_execute(tmp_path, language_images, language):
    vehicle = idle_language_vehicle(tmp_path, language_images, language)
    names = ('RUST', 'ADA', 'F90')
    try:
        vehicle.start()
        vehicle.parameter('TRIK_ENABLE', 1)
        vehicle.parameter('AEROM_LANG', language)
        vehicle.restart(1)
        vehicle.parameter('TRIK1_ID', 72)
        vehicle.mode('CRUISE')
        vehicle.rc(7, 1500)
        vehicle.text('Trick 1 selected (SuperAirShow)')
        vehicle.parameter('AEROM_LANG', 3)
        vehicle.pump(0.4)
        vehicle.rc(7, 2000)
        mark = len(vehicle.texts)
        vehicle.parameter('AEROM_LANG', language)
        vehicle.text(f'Aerobatics {names[language]} selected', after=mark)
        vehicle.pump(0.6)
        assert not any('Trick 1 started' in text for text in vehicle.texts[mark:]), vehicle.diagnostic()
        vehicle.rc(7, 1500)
        vehicle.text('Trick 1 selected (SuperAirShow)', after=mark)
        vehicle.rc(7, 2000)
        vehicle.text('Trick 1 started (SuperAirShow)', after=mark)
        vehicle.parameter('AEROM_LANG', 3)
        vehicle.pump(1.5)
        mark = len(vehicle.texts)
        vehicle.parameter('AEROM_LANG', language)
        vehicle.text(f'Aerobatics {names[language]} selected', after=mark)
        vehicle.pump(0.6)
        assert not any('Trick 1 started' in text for text in vehicle.texts[mark:]), vehicle.diagnostic()
    finally:
        vehicle.stop()


@pytest.mark.parametrize('language', [0, 1, 2])
def test_missing_trick_waits_for_a_new_switch_change(tmp_path, language_images, language):
    vehicle = idle_language_vehicle(tmp_path, language_images, language)
    try:
        vehicle.start()
        vehicle.parameter('TRIK_ENABLE', 1)
        vehicle.parameter('AEROM_LANG', language)
        vehicle.restart(1)
        vehicle.parameter('TRIK1_ID', 73)
        vehicle.mode('CRUISE')
        vehicle.pump(0.3)
        mark = len(vehicle.texts)
        vehicle.rc(7, 1500)
        vehicle.pump(1)
        errors = [text for text in vehicle.texts[mark:] if 'Trick 73 load' in text or 'Aerobatics error' in text]
        assert len(errors) == 1, vehicle.diagnostic()
        assert vehicle.worker(next(iter(vehicle.modules.iterdir())).name) is not None
    finally:
        vehicle.stop()


def test_airshow_language_handoff(tmp_path, language_images):
    """Keep the execute switch held while transferring every pair of languages."""
    files = {path.name: path.read_bytes() for path in language_images.values()}
    vehicle = AerobaticVehicle(tmp_path / 'vehicle', files, {
        'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 32768, 'MOD_CODE_MAXSIZE': 262144,
        'MOD_STACK_MAX': 32768, 'RC7_OPTION': 300, 'RC9_OPTION': 301,
        'RC_OVERRIDE_TIME': 3, 'FLTMODE_CH': 0,
        'LOG_BACKEND_TYPE': 1, 'LOG_DISARMED': 1, 'LOG_FILE_DSRMROT': 0, 'LOG_FILE_BUFSIZE': 128,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    shutil.copyfile(APPLET / 'AirShow.txt', scripts / 'trick72.txt')
    peak = 0
    switches = 0
    try:
        vehicle.start()
        vehicle.parameter('TRIK_ENABLE', 1)
        vehicle.restart(1, expected=3)
        vehicle.parameter('TRIK1_ID', 72)
        vehicle.mode('TAKEOFF')
        vehicle.arm()
        vehicle.until(lambda m: m.get_type() == 'GLOBAL_POSITION_INT' and 30000 <= m.relative_alt <= 40000, 30)
        vehicle.mode('CRUISE')
        vehicle.rc(9, 1000)
        vehicle.rc(7, 1500)
        vehicle.text('Trick 1 selected (SuperAirShow)')
        mark = len(vehicle.texts)
        vehicle.rc(7, 2000)
        vehicle.text('Trick 1 started (SuperAirShow)', after=mark)
        sequence = (1, 2, 0, 2, 1, 0)
        names = ('RUST', 'ADA', 'F90')
        next_switch = time.monotonic() + 0.5
        deadline = time.monotonic() + 240
        while 'Finishing SuperAirShow!' not in vehicle.texts[mark:] and time.monotonic() < deadline:
            message = vehicle.receive()
            if message is not None and message.get_type() == 'NAMED_VALUE_FLOAT' and message.name == 'PERR':
                peak = max(peak, message.value)
                assert peak <= 15, (peak, vehicle.diagnostic())
            assert not any('aborting' in text or 'panic' in text or 'Aerobatics error' in text
                           or 'NavScript time out' in text for text in vehicle.texts[mark:]), vehicle.diagnostic()
            if time.monotonic() >= next_switch:
                language = sequence[switches % len(sequence)]
                switch_mark = len(vehicle.texts)
                vehicle.parameter('AEROM_LANG', language)
                vehicle.text(f'Aerobatics {names[language]} selected', after=switch_mark)
                switches += 1
                next_switch = time.monotonic() + 0.5
        assert switches >= 12 and peak > 0, (switches, peak, vehicle.diagnostic())
        assert 'Finishing SuperAirShow!' in vehicle.texts[mark:], vehicle.diagnostic()
        expected = [line.split(':', 1)[1].strip() for line in (APPLET / 'AirShow.txt').read_text().splitlines()
                    if line.startswith('message:')]
        assert [text for text in vehicle.texts[mark:] if text in expected] == expected, vehicle.diagnostic()
        assert sum('Trick 1 started' in text for text in vehicle.texts[mark:]) == 1, vehicle.diagnostic()
        print(f'AirShow: {switches} live language switches, peak PERR {peak:.3f}m')
        vehicle.arm(False, force=True)
    finally:
        vehicle.stop()
    progress = []
    for file in sorted((vehicle.directory / 'logs').glob('*.BIN')):
        reader = DFReader.DFReader_binary(str(file))
        while (record := reader.recv_msg()) is not None:
            if record.get_type() == 'AETM':
                progress.append((record.T, record.Adt))
    assert len(progress) > 1000
    assert all(a[0] <= b[0] for a, b in zip(progress, progress[1:])), 'Schedule restarted during handoff'
    assert max(dt for _, dt in progress) < 0.2, 'Handoff interrupted the control loop'


def test_mission_language_handoff(tmp_path, language_images):
    """A handoff must retain the mission owner without completing its item."""
    files = {path.name: path.read_bytes() for path in language_images.values()}
    vehicle = AerobaticVehicle(tmp_path / 'vehicle', files, {
        'SCR_ENABLE': 1, 'MOD_HEAP_MAX': 32768, 'MOD_CODE_MAXSIZE': 262144,
        'MOD_STACK_MAX': 32768, 'FLTMODE_CH': 0,
    })
    scripts = vehicle.directory / 'scripts'
    scripts.mkdir()
    (scripts / 'driver.lua').write_text('''local stage=0
local function update()
 if not param:get("AEROM_LANG") or not ahrs:get_relative_position_NED_origin() then return update,100 end
 if stage==0 then
  assert(param:set("AEROM_ALT_ABORT",1000))
  assert(param:set("AEROM_MIS_ANGLE",90))
  assert(mission:clear())
  local m=mavlink_mission_item_int_t()
  m:command(16); m:x(-353632610); m:y(1491652300); m:z(40); m:frame(3)
  assert(mission:set_item(0,m))
  m:command(42702); m:param1(7); m:param3(10000); m:param4(1); m:x(0); m:y(0)
  assert(mission:set_item(1,m))
  m:command(16); m:x(-353632610); m:y(1491652300)
  assert(mission:set_item(2,m))
 elseif stage==1 then
  assert(vehicle:set_mode(10))
 elseif stage==2 then
  assert(mission:set_current_cmd(1))
 end
 stage=stage+1
 return update,100
end
return update,100
''')
    try:
        vehicle.start()
        vehicle.text('Starting Axial Roll!', timeout=20)
        mark = len(vehicle.texts)
        for language in (1, 2, 0, 2, 1, 0):
            switch_mark = len(vehicle.texts)
            vehicle.parameter('AEROM_LANG', language)
            vehicle.text(f'Aerobatics {("RUST", "ADA", "F90")[language]} selected', after=switch_mark)
            vehicle.pump(0.3)
            vehicle.until(lambda m: m.get_type() == 'MISSION_CURRENT' and m.seq == 1)
        assert not any('Starting' in text or 'Finishing' in text or 'Aerobatics error' in text
                       or 'aborting' in text or 'NavScript time out' in text
                       for text in vehicle.texts[mark:]), vehicle.diagnostic()
        vehicle.link.mav.mission_set_current_send(1, 1, 2)
        vehicle.until(lambda m: m.get_type() == 'MISSION_CURRENT' and m.seq == 2)
        vehicle.parameter('AEROM_LANG', 1)
        vehicle.pump(1)
        assert not any('Starting' in text for text in vehicle.texts[mark:]), vehicle.diagnostic()
        # A complete restart clears the shared block and permits the same item again.
        vehicle.restart(1, expected=3)
        mark = len(vehicle.texts)
        vehicle.link.mav.mission_set_current_send(1, 1, 1)
        vehicle.text('Starting Axial Roll!', after=mark)
        # Let the command deadline expire while no language is selected.
        vehicle.parameter('AEROM_LANG', 3)
        vehicle.text('NavScript time out', after=mark)
        mark = len(vehicle.texts)
        vehicle.parameter('AEROM_LANG', 2)
        vehicle.pump(1)
        assert not any('Starting' in text or 'Finishing' in text or 'Aerobatics error' in text
                       for text in vehicle.texts[mark:]), vehicle.diagnostic()
    finally:
        vehicle.stop()

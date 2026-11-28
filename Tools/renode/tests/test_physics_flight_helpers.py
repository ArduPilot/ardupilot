#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Unit tests for manual Renode physics flight options."""

import importlib.util
import re
import sys

from pathlib import Path
from types import SimpleNamespace

import pytest

MODULE_PATH = Path(__file__).resolve().parent / 'test_physics_flight.py'
SPEC = importlib.util.spec_from_file_location(
    'renode_test_physics_flight', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
physics_flight = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = physics_flight
SPEC.loader.exec_module(physics_flight)


def options(**overrides):
    values = {
        'renode': None,
        'data_cache': None,
        'usb': False,
        'gdb': False,
        'interactive': False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_add_launch_options():
    command = ['run.py']
    physics_flight.add_launch_options(
        command,
        options(
            renode='/opt/renode', data_cache='/tmp/data',
            usb=True, gdb=True),
    )
    assert command == [
        'run.py', '--unthrottled', '--renode', '/opt/renode',
        '--data-cache', '/tmp/data', '--usb', '--gdb',
    ]

    interactive_command = ['run.py']
    physics_flight.add_launch_options(
        interactive_command, options(interactive=True))
    assert interactive_command == ['run.py']


@pytest.mark.parametrize(
    'builder,board,target',
    (
        (physics_flight.build_plane, 'MatekH743', 'plane'),
        (physics_flight.build_copter, 'KakuteF4', 'copter'),
        (physics_flight.build_rpi_uavfc_copter, 'RPI_UAVFC', 'copter'),
        (physics_flight.build_quadplane, 'CubeOrangePlus', 'plane'),
    ),
)
def test_gdb_build_adds_debug_symbols(
        monkeypatch, tmp_path, builder, board, target):
    commands = []
    monkeypatch.setattr(
        physics_flight.subprocess, 'run',
        lambda command, **kwargs: commands.append((command, kwargs)))

    builder(tmp_path, debug_symbols=True)

    configure, configure_kwargs = commands[0]
    assert configure[:4] == ['./waf', 'configure', '--board', board]
    assert '-g' in configure
    assert '--debug' not in configure
    assert configure_kwargs == {'cwd': tmp_path, 'check': True}
    assert commands[1][0] == ['./waf', target]


def test_gdb_rejects_non_elf_firmware(tmp_path):
    firmware = tmp_path / 'firmware.bin'
    firmware.write_bytes(b'raw firmware')

    with pytest.raises(SystemExit) as error:
        physics_flight.main([
            'plane', '--firmware', str(firmware), '--gdb',
        ])

    assert error.value.code == 2


def test_resc_boot_script_sets_what_the_board_script_reads(tmp_path):
    root = Path(__file__).resolve().parents[3]
    profile = physics_flight.COPTER_PROFILES['rpi-uavfc-copter']
    board_script = (root / profile['resc']).read_text()
    elf = tmp_path / 'arducopter'

    script = physics_flight.write_resc_boot_script(
        root, profile, tmp_path, elf, 5762, 9002,
        physics_flight.CANBERRA, profile['rate'])

    sd_image = tmp_path / 'sdcard.img'
    assert script == tmp_path / 'boot.resc'
    assert script.read_text().splitlines() == [
        '$repo = @%s' % root,
        '$elf = @%s' % elf,
        '$sdcard = @%s' % sd_image,
        '$serial_port = 5762',
        'include @%s' % (root / profile['resc']),
        'mach set 0',
        'logLevel 3',
        'sysbus.physics Connect 9002 "bfx" -35.3632610 149.1652300 584.0 353.0 200',
        'start',
    ]
    # only variables the board script declares with a ?= default
    declared = set(re.findall(r'^\$(\w+)\?=', board_script, re.MULTILINE))
    assert {'repo', 'elf', 'sdcard', 'serial_port'} <= declared
    # a card of the size the board script mounts
    mounted = re.search(
        r'^machine AP_SpiSdCardFromFile \$sdcard \S+ \d+ (0x[0-9A-Fa-f]+)',
        board_script, re.MULTILINE)
    assert mounted is not None
    assert sd_image.stat().st_size == int(mounted.group(1), 16)


@pytest.mark.parametrize('flag', ('usb', 'gdb', 'firmware'))
def test_resc_profile_refuses_options_it_cannot_honour(
        monkeypatch, tmp_path, flag):
    monkeypatch.setattr(
        physics_flight, 'build_physics',
        lambda root: pytest.fail('built before refusing'))
    firmware = tmp_path / 'arducopter.bin'
    firmware.write_bytes(b'raw firmware')
    args = options(skip_build=False, firmware=None)
    setattr(args, flag, {'usb': True, 'gdb': True, 'firmware': firmware}[flag])

    with pytest.raises(RuntimeError, match='RPI_UAVFC Copter'):
        physics_flight.run_copter(
            args, tmp_path, tmp_path,
            physics_flight.COPTER_PROFILES['rpi-uavfc-copter'])


def test_wait_interactive_reports_endpoints_and_stops_on_interrupt(
        monkeypatch, capsys):
    checks = []
    args = options(usb=True, gdb=True)
    monkeypatch.setattr(
        physics_flight.common, 'check_process',
        lambda process, log: checks.append((process, log)))
    monkeypatch.setattr(
        physics_flight, 'check_sidecar',
        lambda process, log: checks.append((process, log)))
    monkeypatch.setattr(
        physics_flight.time, 'sleep',
        lambda _delay: (_ for _ in ()).throw(KeyboardInterrupt()))

    physics_flight.wait_interactive(
        args, 'Test Board', 5762, 'renode', 'renode.log',
        'physics', 'physics.log',
        SimpleNamespace(poll=lambda: None), 'usbip.log')

    output = capsys.readouterr().out
    assert 'MAVLink: tcp:127.0.0.1:5762' in output
    assert 'USB/IP: attaching automatically (status in usbip.log)' in output
    assert 'halted at reset' in output
    assert checks == [
        ('renode', 'renode.log'),
        ('physics', 'physics.log'),
    ]


def test_start_usb_helper(monkeypatch, tmp_path):
    launches = []
    expected_process = SimpleNamespace()

    def popen(command, **kwargs):
        launches.append((command, kwargs))
        return expected_process

    monkeypatch.setattr(physics_flight.subprocess, 'Popen', popen)
    root = tmp_path / 'root'
    output = tmp_path / 'output'
    output.mkdir()

    process, log_path = physics_flight.start_usb_helper(
        options(usb=True), root, output)

    assert process is expected_process
    assert log_path == output / 'usbip.log'
    command, kwargs = launches[0]
    assert command == [
        physics_flight.sys.executable,
        str(root / 'Tools' / 'renode' / 'usbip_attach.py'),
        '--port', '3240',
    ]
    assert kwargs['cwd'] == root
    assert kwargs['stderr'] is physics_flight.subprocess.STDOUT
    assert kwargs['start_new_session'] is True
    assert kwargs['stdout'].name == str(log_path)

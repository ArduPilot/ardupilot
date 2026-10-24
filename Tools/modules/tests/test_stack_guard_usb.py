# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Destructive stack-guard tests through the firmware USB monitor in Renode."""

import contextlib
import os
import signal
import subprocess
import sys
import time

from pathlib import Path

import pytest

from elftools.elf.elffile import ELFFile
from test_gdb_usb_runtime import ROOT
from test_gdb_usb_runtime import free_port
from test_usb_debug_firmware import monitor_read
from test_usb_debug_firmware import monitor_write
from test_usb_debug_firmware import read_memory
from test_usb_debug_firmware import resume
from test_usb_debug_firmware import write_memory
from usb_debug_transport import GDB
from usb_debug_transport import USB
from usb_debug_transport import gdb


@pytest.mark.parametrize('mode', [0, 1, 2, 3, 4, 5, 6])
def test_usb_stack_guard(tmp_path, monkeypatch, mode):
    firmware = Path(os.environ.get('AP_STACK_USB_ELF', ROOT / 'build/CubeOrangePlus/examples/StackGuard'))
    renode = os.environ.get('RENODE')
    if not renode or not firmware.is_file():
        pytest.skip('build CubeOrangePlus StackGuard with --enable-USB-debug --enable-STACK_GUARD; set AP_STACK_USB_ELF')
    with firmware.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        assert symbols.get_symbol_by_name('ap_stack_guard_recover')
        assert symbols.get_symbol_by_name('USB_Debug_Handler')
        names = ['result', 'cycles', 'before', 'after', 'mode', 'heartbeat', 'start', 'faults']
        addresses = {name: symbols.get_symbol_by_name('stack_guard_test_' + name)[0]['st_value'] for name in names}
        worker = symbols.get_symbol_by_name('_ZN17StackGuardExample6workerEv')[0]['st_value'] & ~1
    usb_port, monitor_port, uart_port = free_port(), free_port(), free_port()
    monkeypatch.setenv('USB_DEBUG_MONITOR_PORT', str(monitor_port))
    overlay = tmp_path / 'fpb.repl'
    overlay.write_text('fpb: Miscellaneous.AP_CortexM_FPB @ sysbus 0xE0002000\n    cpu: cpu\n')
    command = [sys.executable, str(ROOT / 'Tools/renode/run.py'), 'CubeOrangePlus', '--elf', str(firmware),
               '--renode', renode, '--state-dir', str(tmp_path / 'state'), '--usb', '--usbip-port', str(usb_port),
               '--port', str(monitor_port), '--uart-port', str(uart_port), '--unthrottled',
               '--exec', f'include @{ROOT / "Tools/renode/peripherals/cpu/AP_CortexM_FPB.cs"}',
               '--exec', f'machine LoadPlatformDescription @{overlay}']
    usb = None
    with (tmp_path / 'renode.log').open('w') as log:
        emulator = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                    start_new_session=True,
                                    env=dict(os.environ, TMPDIR=str(tmp_path), XDG_CONFIG_HOME=str(tmp_path / 'xdg')))
        try:
            usb = USB(usb_port)
            usb.write(4, gdb.TRIGGER)
            deadline = time.monotonic() + 20
            while usb.running and time.monotonic() < deadline:
                time.sleep(.1)
            assert not usb.running, 'firmware did not enter the USB monitor'
            usb.close()
            usb = USB(usb_port)
            debug = GDB(usb)
            assert b'ap-usb-debug+' in debug.packet(b'qSupported')
            write_memory(debug, addresses['mode'], mode.to_bytes(4, 'little'))
            assert debug.packet(f'Z1,{worker:x},2'.encode()) == b'OK'
            write_memory(debug, addresses['start'], (1).to_bytes(4, 'little'))
            assert resume(debug).startswith(b'T05')
            heartbeat_before = int.from_bytes(read_memory(debug, addresses['heartbeat'], 4), 'little')

            # A debugger probe into the active guard must return E01 without
            # terminating the worker, disabling its MPU region or leaking fault
            # status into the application's subsequent overflow classification.
            # PPB registers are intentionally not in the debugger's memory
            # whitelist. Inspect the MPU through Renode's monitor instead.
            rnr = monitor_read(0xe000ed98)
            monitor_write(0xe000ed98, 7)
            region = (monitor_read(0xe000ed9c), monitor_read(0xe000eda0))
            base = region[0] & ~31
            assert region[1] & 1
            assert debug.packet(f'm{base:x},1'.encode()) == b'E01'
            assert debug.packet(f'M{base:x},1:01'.encode()) == b'E01'
            assert (monitor_read(0xe000ed9c), monitor_read(0xe000eda0)) == region
            assert monitor_read(0xe000ed28) == 0 and monitor_read(0xe000ed2c) == 0
            assert read_memory(debug, addresses['faults'], 4) == bytes(4)
            monitor_write(0xe000ed98, rnr)
            assert debug.packet(f'z1,{worker:x},2'.encode()) == b'OK'

            if mode in (5, 6):
                # Unattributable / critical-section faults must not be treated
                # as recovered overflows. A valid frame remains debuggable.
                assert resume(debug).startswith(b'T0b')
                assert read_memory(debug, addresses['faults'], 4) == bytes(4)
                assert read_memory(debug, addresses['cycles'], 4) == bytes(4)
                status = bytes.fromhex(debug.packet(b'qRcmd,6661756c74').decode()).decode()
                assert 'CFSR=00000082' in status
            else:
                debug.write(gdb.rsp_packet(b'c'))
                deadline = time.monotonic() + 30
                while monitor_read(addresses['result']) != 0x600DF00D:
                    assert time.monotonic() < deadline, (tmp_path / 'renode.log').read_text()
                assert debug.stop().startswith(b'T02')
                values = {name: int.from_bytes(read_memory(debug, address, 4), 'little')
                          for name, address in addresses.items()}
                # One warmup fault, then 100 create/fault/join/reap cycles.
                assert values['cycles'] == 100 and values['faults'] == 101, values
                assert values['before'] == values['after'], values
                assert values['heartbeat'] > heartbeat_before, values
        finally:
            if usb:
                usb.close()
            with contextlib.suppress(ProcessLookupError):
                os.killpg(emulator.pid, signal.SIGKILL)
            emulator.wait()

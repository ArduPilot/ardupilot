# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run real GDB through the firmware USB monitor, not Renode's GDB stub."""

import contextlib
import os
import signal
import socket
import subprocess
import sys
import threading
import time

from pathlib import Path
from types import SimpleNamespace

import pytest

import build

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'Tools/renode'), str(ROOT / 'Tools/renode/tests')]
import fat_image  # noqa: E402

from usb_debug_transport import GCS  # noqa: E402
from usb_debug_transport import GDB  # noqa: E402
from usb_debug_transport import USB  # noqa: E402
from usb_debug_transport import gdb as usb_debug  # noqa: E402


def free_port():
    with socket.socket() as stream:
        stream.bind(('127.0.0.1', 0))
        return stream.getsockname()[1]


def test_usb_module_debugging(tmp_path):
    firmware = Path(os.environ.get('AP_MODULE_USB_ELF', ROOT / 'build/CubeOrangePlus/bin/arduplane'))
    debugger = os.environ.get('AP_MODULE_ARM_GDB')
    renode = os.environ.get('RENODE')
    if not debugger or not renode or not firmware.is_file():
        pytest.skip('set AP_MODULE_USB_ELF, AP_MODULE_ARM_GDB (Python enabled), and patched RENODE')
    module = build.build(ROOT / 'libraries/AP_Module/tests/modules/debug.rs', tmp_path / 'debug.apmod',
                         'thumbv7em-none-eabihf', 16384, debug=True, opt_level='1')
    state = tmp_path / 'state'
    image = fat_image.create_image(state / 'sdcard.img', 512 * 1024 * 1024, sectors_per_cluster=8)
    with fat_image.PyFatFS(str(image)) as fs:
        fs.makedirs('/APM/modules', recreate=True)
        with fs.openbin('/APM/modules/debug.apmod', 'w') as output:
            output.write(module.read_bytes())
    overlay = tmp_path / 'fpb.repl'
    overlay.write_text('fpb: Miscellaneous.AP_CortexM_FPB @ sysbus 0xE0002000\n    cpu: cpu\n')
    usb_port, monitor_port, uart_port = free_port(), free_port(), free_port()
    command = [sys.executable, str(ROOT / 'Tools/renode/run.py'), 'CubeOrangePlus', '--elf', str(firmware),
               '--renode', renode, '--state-dir', str(state), '--usb', '--usbip-port', str(usb_port),
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
            gcs = GCS(usb)
            gcs.recv('HEARTBEAT', 60)
            gcs.m.param_set_send(1, 1, b'MOD_ENABLE', 1, 9)
            while gcs.recv('PARAM_VALUE', 20).param_id != 'MOD_ENABLE':
                pass
            gcs.m.command_long_send(1, 1, 246, 0, 1, 0, 0, 0, 0, 0, 0)
            deadline = time.monotonic() + 20
            while usb.running and time.monotonic() < deadline:
                time.sleep(.1)
            usb.close()
            usb = USB(usb_port)
            gcs = GCS(usb)
            deadline = time.monotonic() + 60
            while True:
                message = gcs.recv('STATUSTEXT', max(1, deadline - time.monotonic()))
                if 'Module: loaded 1 modules' in message.text:
                    break
                assert time.monotonic() < deadline, message
            usb.write(4, usb_debug.TRIGGER)
            deadline = time.monotonic() + 20
            while usb.running and time.monotonic() < deadline:
                time.sleep(.1)
            usb.close()
            usb = USB(usb_port)
            debug = GDB(usb)
            assert b'ap-usb-debug+' in debug.packet(b'qSupported')
            with socket.socket() as server:
                server.bind(('127.0.0.1', 0))
                server.listen(1)
                server.settimeout(30)

                def serve():
                    client, _ = server.accept()
                    with client:
                        usb_debug.bridge(client, debug)

                bridge = threading.Thread(target=serve, daemon=True)
                bridge.start()
                expressions = [
                    'set confirm off', 'set remotetimeout 20',
                    'apmod list', 'break apmod_debug_tick', 'continue',
                    'next', 'print label', 'print *values', 'print debug::APMOD_DEBUG_COUNT',
                    'print debug::APMOD_DEBUG_MAGIC', 'bt 8', 'next',
                    'set language c++', "set var 'AP_Module::Runtime::_singleton'->_restart._value = 1",
                    'set language auto', 'disable 1', 'tbreak AP_Module::Module::start()', 'continue',
                    'break apmod_debug_tick', 'continue', 'next', 'print label',
                    'print debug::APMOD_DEBUG_COUNT', 'apmod clear', 'detach',
                ]
                args = usb_debug.gdb_command(SimpleNamespace(
                    gdb=debugger, elf=firmware, gdb_server=f'127.0.0.1:{server.getsockname()[1]}', port=None,
                    batch=True, apmod=[module], ex=expressions))
                with (tmp_path / 'gdb.log').open('w') as output:
                    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                    try:
                        process.wait(timeout=90)
                    finally:
                        with contextlib.suppress(ProcessLookupError):
                            os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                text = (tmp_path / 'gdb.log').read_text()
                assert process.returncode == 0, text
                assert 'apmod: debug.apmod RX=0x24' in text, text
                assert '"apmod debug"' in text and 'Vec(size=3)' in text, text
                assert '305419896' in text and 'debug.rs:' in text, text
                assert text.count('apmod: debug.apmod RX=') == 2, text
                assert 'apmod: removing breakpoint' in text and 'AP_Module::Module::run' in text, text
                assert 'corrupt stack' not in text, text
                assert 'lifecycle error' not in text and 'Cannot access memory' not in text, text
                bridge.join(5)
        finally:
            if usb:
                usb.close()
            with contextlib.suppress(ProcessLookupError):
                os.killpg(emulator.pid, signal.SIGKILL)
            emulator.wait()

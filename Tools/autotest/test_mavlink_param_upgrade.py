#!/usr/bin/env python3
"""Check Copter system-ID upgrades using persistent EEPROM fixtures.

Run after ./waf copter:
    python3 Tools/autotest/test_mavlink_param_upgrade.py

Requires pymavlink with 32-bit system ID support.

AP_FLAKE8_CLEAN
"""

import argparse
from pathlib import Path
import struct
import subprocess
import tempfile
import time

from pymavlink import mavutil


def entry(key, group, value, width=16):
    # AP_Param revision 6: key_low:8, type:5, key_high:1, group:18.
    param_type = 2 if width == 16 else 3
    header = (key & 255) | (param_type << 8) | ((key >> 8) << 13) | (group << 14)
    return struct.pack('<I' + ('h' if width == 16 else 'i'), header, value)


def fixture(current=None, legacy=None, width=16, legacy_first=False):
    # Copter storage keys: FORMAT_VERSION=0 (120), old SYSIDs=112/113,
    # MAV group=260; GCS group elements SYSID=1, GCS_SYSID=2, HI=5.
    old = b'' if legacy is None else b''.join(entry(key, 0, value) for key, value in zip((112, 113), legacy))
    new = b'' if current is None else b''.join(
        entry(260, group, value, width) for group, value in zip((1, 2, 5), current))
    params = b'PA\x06\x00' + entry(0, 0, 120) + (old + new if legacy_first else new + old) + b'\xff' * 4
    # All fixture entries fit in the first StorageParam region (1536 bytes).
    return params.ljust(32768, b'\x00')


def check_boot(binary, directory, expected, instance):
    with (directory / 'sitl.log').open('a') as log:
        process = subprocess.Popen(
            [str(binary), '--model', '+', '--speedup', '5', '-I', str(instance)],
            cwd=directory, stdout=log, stderr=subprocess.STDOUT)
        link = None
        try:
            link = mavutil.mavlink_connection('tcp:127.0.0.1:%u' % (5760 + 10 * instance), retries=30)
            heartbeat = link.wait_heartbeat(timeout=30)
            if heartbeat is None:
                raise AssertionError('No heartbeat')
            source = heartbeat.get_srcSystem()
            actual = []
            for name in ('MAV_SYSID', 'MAV_GCS_SYSID', 'MAV_GCS_SYSID_HI'):
                value = None
                for attempt in range(10):
                    link.mav.param_request_read_send(source, 1, name.encode(), -1)
                    deadline = time.monotonic() + 1
                    while time.monotonic() < deadline:
                        message = link.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                        if message is not None and message.param_id == name:
                            value = message.param_value
                            break
                    if value is not None:
                        break
                if value is None:
                    raise AssertionError('No response for ' + name)
                actual.append(value)
            if tuple(actual) != expected or source != expected[0]:
                raise AssertionError('expected %s, got %s, heartbeat sysid=%u' % (expected, actual, source))
            # Let the deferred parameter saves reach eeprom.bin before reboot.
            time.sleep(1)
        finally:
            if link is not None:
                link.close()
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path('build/sitl/bin/arducopter'))
    parser.add_argument('--instance', type=int, default=120)
    args = parser.parse_args()
    binary = args.binary.resolve(strict=True)
    cases = [
        ('current-before-legacy', fixture((7, 247, 250), (3, 243)), (7, 247, 250)),
        ('legacy-before-current', fixture((7, 247, 250), (3, 243), legacy_first=True), (7, 247, 250)),
        ('saved-defaults', fixture((1, 255, 0), (3, 243)), (1, 255, 0)),
        ('legacy-only', fixture(legacy=(3, 243)), (3, 243, 0)),
        ('current-only', fixture((7, 247, 250)), (7, 247, 250)),
        ('already-int32', fixture((70000, 80000, 90000), (3, 243), width=32), (70000, 80000, 90000)),
        ('defaults', fixture(), (1, 255, 0)),
    ]
    failures = []
    for name, storage, expected in cases:
        with tempfile.TemporaryDirectory(prefix='mavlink-upgrade-') as temp:
            directory = Path(temp)
            (directory / 'eeprom.bin').write_bytes(storage)
            try:
                for boot in range(2):
                    check_boot(binary, directory, expected, args.instance)
                print('PASS: %s (upgrade and reboot)' % name, flush=True)
            except (AssertionError, OSError) as error:
                failures.append(name)
                print('FAIL: %s: %s' % (name, error), flush=True)
                print((directory / 'sitl.log').read_text(), flush=True)
    if failures:
        raise SystemExit('Failed cases: ' + ', '.join(failures))


if __name__ == '__main__':
    main()

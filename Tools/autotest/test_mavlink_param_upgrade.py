#!/usr/bin/env python3
"""Check Copter system-ID upgrades using persistent EEPROM fixtures.

Run after ./waf copter:
    python3 Tools/autotest/test_mavlink_param_upgrade.py

Requires pymavlink with 32-bit system ID support.

AP_FLAKE8_CLEAN
"""

import argparse
import struct
import tempfile

from pathlib import Path

import pexpect

from test_param_upgrade import TestParamUpgradeTestSuite

from vehicle_test_suite import ErrorException
from vehicle_test_suite import NotAchievedException


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


class MAVLinkParamUpgradeTestSuite(TestParamUpgradeTestSuite):
    def __init__(self, binary, directory, expected, instance):
        super().__init__(str(binary))
        self.directory = directory
        self.expected = dict(zip(('MAV_SYSID', 'MAV_GCS_SYSID', 'MAV_GCS_SYSID_HI'), expected))
        self.instance = instance

    def adjust_ardupilot_port(self, port):
        return port + 10 * self.instance

    def sysid_thismav(self):
        # Follow the actual heartbeat ID so a rollback reports the wrong
        # parameter value instead of timing out waiting for the expected ID.
        return self.mav.target_system if self.mav is not None else 1

    def run(self):
        try:
            self.start_SITL(
                model='X',  # the EEPROM fixtures are specific to Copter
                sitl_home="1,1,1,1",
                wipe=False,
                cwd=self.directory,
                customisations=['-I', str(self.instance)],
            )
            self.get_mavlink_connection_going()
            self.assert_parameter_values(self.expected)
            heartbeat = self.wait_heartbeat()
            if heartbeat.get_srcSystem() != self.expected['MAV_SYSID']:
                raise NotAchievedException("Heartbeat system ID does not match MAV_SYSID")
            self.delay_sim_time(2, reason="EEPROM write to complete")
        finally:
            if self.mav is not None:
                self.mav.close()
                self.mav = None
            if getattr(self, 'sitl', None) is not None:
                self.stop_SITL()


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
                    suite = MAVLinkParamUpgradeTestSuite(binary, directory, expected, args.instance)
                    suite.run()
                print('PASS: %s (upgrade and reboot)' % name, flush=True)
            except (ErrorException, OSError, pexpect.TIMEOUT, pexpect.EOF) as error:
                failures.append(name)
                print('FAIL: %s: %s' % (name, error), flush=True)
    if failures:
        raise SystemExit('Failed cases: ' + ', '.join(failures))


if __name__ == '__main__':
    main()

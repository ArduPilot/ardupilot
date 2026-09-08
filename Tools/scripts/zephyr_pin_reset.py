#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Hardware-reset the target by pulsing nRST through the debug probe -
with ZERO DAP transactions.

Exists because a wedged SWD DAP (e.g. after pyocd's default discovery
touches the dormant CM4's AHB-AP#1 on mr_vmu_rt1176, which WAITs forever)
blocks every normal `pyocd reset` - the connect fails before the reset is
ever issued - and the historical remedy was a HUMAN power-cycle. The probe
can drive the reset PIN without any SWD traffic, which both reboots the
target and clears the wedged DP.

This is the autonomous-flash-recipe reset:
    uploader.py <apj> &  sleep 3;  zephyr_pin_reset.py

Usage: Tools/scripts/zephyr_pin_reset.py [probe-uid]   (default MCU-Link-MR)
"""
import sys
import time

from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session

UID = sys.argv[1] if len(sys.argv) > 1 else 'JUHP1E4TMRVGD'


def main():
    probe = ConnectHelper.choose_probe(unique_id=UID)
    if probe is None:
        print('no probe %s' % UID)
        return 1
    Session(probe)  # binds probe.session; no target init performed
    probe.open()
    probe.assert_reset(True)
    time.sleep(0.3)
    probe.assert_reset(False)
    probe.close()
    print('nRST pulsed via %s - zero DAP transactions' % UID)
    return 0


if __name__ == '__main__':
    sys.exit(main())

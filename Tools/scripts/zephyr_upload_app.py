#!/usr/bin/env python3
# encoding: utf-8

"""
Upload an ArduPilot application .apj to a Zephyr board via uploader.py, with
the serial port PINNED to that board.

WHY THIS EXISTS
---------------
uploader.py's default port list starts with '/dev/serial/by-id/usb-Ardu*',
and send_reboot() fires MAVLink reboot bytes at EVERY port that matches. On a
bench with more than one ArduPilot board attached, a bare uploader.py run
reboots the other boards too - including ones another person or session is
mid-operation on. (This happened on 2026-08-16: bare runs were knocking an
rp2350 into its bootloader.)

Pinning with a GLOB - not a raw tty - keeps uploader.py's own bootloader-catch
behaviour intact: the glob still matches the '-BL' device the instant it
enumerates, and the list is re-globbed on each pass. Pinning a raw tty like
/dev/ttyACM1 breaks that and makes the upload hang forever waiting for a
bootloader window it can no longer see.

USAGE
-----
    zephyr_upload_app.py                      # default board + default .apj
    zephyr_upload_app.py <image.apj>
    zephyr_upload_app.py <image.apj> <board>

If the upload sits at "Attempting reboot..." the board is not in a state where
it will reboot itself into the bootloader (a wedged or absent app). Open the
bootloader window externally while this is retrying:

    python3 Tools/scripts/zephyr_pin_reset.py

which pulses nRST through the probe with zero DAP transactions.

AP_FLAKE8_CLEAN
"""

import os
import subprocess
import sys

# Per-board USB serial glob. Keep these as GLOBS so the bootloader's own
# '-BL' identity is matched too.
BOARD_PORT_GLOBS = {
    'mr_vmu_rt1176': '/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176*',
}

DEFAULT_BOARD = 'mr_vmu_rt1176'


def main():
    board = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_BOARD
    if board not in BOARD_PORT_GLOBS:
        sys.exit("no port glob known for board '%s' - add one to "
                 "BOARD_PORT_GLOBS" % board)
    port_glob = BOARD_PORT_GLOBS[board]

    default_apj = os.path.join('build', board, 'zephyr_upload.apj')
    apj = sys.argv[1] if len(sys.argv) > 1 else default_apj
    if not os.path.exists(apj):
        sys.exit("no such image: %s (build it first)" % apj)

    srcroot = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    uploader = os.path.join(srcroot, 'Tools', 'scripts', 'uploader.py')
    if not os.path.exists(uploader):
        uploader = os.path.join('Tools', 'scripts', 'uploader.py')

    cmd = [sys.executable, '-u', uploader, '--port', port_glob, apj]
    print("uploading %s to %s" % (apj, board))
    print("  port pinned to: %s" % port_glob)
    return subprocess.call(cmd)


if __name__ == '__main__':
    sys.exit(main())

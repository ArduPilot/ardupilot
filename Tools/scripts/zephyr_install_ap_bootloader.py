#!/usr/bin/env python3
# encoding: utf-8

"""
Install AP_Bootloader over a resident PX4 bootloader - WITHOUT pressing BOOT0.

WHY THIS WORKS WITHOUT BOOT0
----------------------------
The long-standing note on this board said LinkServer can only program the
Macronix Octal-DDR NOR after a power-on with BOOT0 held. Measured 2026-08-16,
that is only half true, and the half that matters is the other half:

  - After ZEPHYR's flash driver has reconfigured FlexSPI into Octal-DDR for
    its own XIP use, LinkServer's SFDP auto-detection fails ("0B at
    0x30000000" / "Flash Driver describes no sectors"). That is the state the
    old note was written in, and BOOT0 is genuinely needed to escape it.

  - With the PX4 BOOTLOADER resident (a fresh board, or any board sitting in
    its bootloader), FlexSPI is still in a state LinkServer understands. SFDP
    detection succeeds - "64MB = 1024*64K at 0x30000000" - and programming
    works normally.

So on a board that still has the PX4 bootloader, this is a plain SWD flash and
needs no straps, no disassembly, and no serial downloader.

SCOPE: THIS DOES *NOT* UPGRADE AN EXISTING AP_Bootloader
-------------------------------------------------------
Measured 2026-08-16 on mr_vmu_rt1176, immediately after this tool had
successfully replaced a PX4 bootloader on the same board:

  - PX4 bootloader resident  -> works (that is the case this tool is for).
  - AP_Bootloader resident   -> FAILS, and not at flash detection. SFDP
    succeeds ("Inspected v.2 External Flash Device ... MXIC_OPI"), then:
        Flash Driver V.2 startup failed - Em(12).
        Target rejected debug access at location 0x2000A7C0
    LinkServer stages its flash algorithm in DTCM at ~0x2000A7C0, and that
    address is not accessible while our firmware is resident. The .cfx has
    that load address baked in - adding RAM regions to the device profile
    does NOT relocate it (tried). Clearing IOMUXC_GPR_GPR16 bit 2, the
    FlexRAM-banking restore used by rt1176_enter_isp.py, does not help
    either: GPR16 already reads 0 and the address is still rejected, so the
    banking bit is not the whole story. Root cause not established.

Note that "AP_Bootloader is PX4-compatible" is true of the SERIAL protocol
(uploader.py drives both) and is irrelevant here - LinkServer never speaks
that protocol; it is pure SWD plus the flash driver.

For AP_Bootloader -> AP_Bootloader upgrades use the in-app path instead:
MAV_CMD_FLASH_BOOTLOADER (magic 290876) flashes the ROMFS-embedded
bootloader from the running application. It is hardware-verified on this
port, needs neither BOOT0 nor a debug probe, and is the intended mechanism.
This SWD tool exists for the case that one cannot cover: a board whose
resident bootloader is not ours and whose application therefore never boots.

WHAT IT DOES
------------
  1. Sanity-checks the AP_Bootloader image: fits the 128 KB slot, carries the
     NXP FCB tag "FCFB" at +0x400, and has a sane vector pair. An image
     failing these is not a bootloader and is refused - writing a bad one over
     the only bootloader is how boards get bricked.
  2. Backs up the existing bootloader region to a file, so the PX4 bootloader
     can be put back (flash the backup with this same tool).
  3. Programs at 0x30000000 WITHOUT a mass erase, so only the sectors the
     image occupies are erased. The application slot at +0x20000 is left
     intact - which means a board that already has an app keeps it.
  4. Verifies, and reports what to do next.

Note the deliberate absence of the '-e' mass erase used for full-chip installs:
that erases all 64 MB (~167 s, no progress output) and would take the app with
it.

USAGE
-----
    zephyr_install_ap_bootloader.py                       # default BL image
    zephyr_install_ap_bootloader.py <bootloader.bin>
    zephyr_install_ap_bootloader.py <bootloader.bin> <backup.bin>

Afterwards, reset the board:

    python3 Tools/scripts/zephyr_pin_reset.py

It should enumerate as '<BOARD>-BL' and then hand off to the application.

AP_FLAKE8_CLEAN
"""

import glob
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time


def _find_linkserver():
    """LinkServer path: $LINKSERVER, else any /usr/local/LinkServer_*, else PATH."""
    env = os.environ.get("LINKSERVER")
    if env:
        return env
    found = sorted(glob.glob("/usr/local/LinkServer_*/LinkServer"))
    if found:
        return found[-1]
    return shutil.which("LinkServer") or "LinkServer"


LINKSERVER = _find_linkserver()
# Pin the MCU-Link by serial. Multiple CMSIS-DAP probes are often attached at
# once, and an unpinned LinkServer will happily select one of them and erase
# the wrong target. Override with $AP_PROBE_SERIAL for your own probe; the
# default is the author's, so on another bench this fails to find a probe
# rather than flashing the wrong one.
PROBE_SERIAL = os.environ.get("AP_PROBE_SERIAL", "JUHP1E4TMRVGD")

FLASH_BASE = 0x30000000
BL_SLOT_SIZE = 128 * 1024          # FLASH_BOOTLOADER_LOAD_KB
FCB_TAG_OFFSET = 0x400
FCB_TAG = b'FCFB'
VECTORS_OFF = 0x2000               # vector table offset within the image

DEFAULT_BL = "Tools/bootloaders/mr_vmu_rt1176_bl.bin"

DEVICE_CONFIG = {
    "copyright": "Copyright 2026 NXP",
    "license": "SPDX-License-Identifier: BSD-3-Clause",
    "version": "2.0.0",
    "vendor": "NXP",
    "device-dataset": [
        {
            "board": {"id": "MIMXRT1170-EVK-CM7-ONLY"},
            "device": {
                "id": "MIMXRT1176xxxxx",
                "name": "MIMXRT1176",
                "family": "MIMXRT1170",
                "memory": [
                    {
                        "location": "0x30000000",
                        "size": "0x04000000",
                        "type": "ExtFlash",
                        "flash-driver": "MIMXRT1170_SFDP_MXIC_OPI.cfx",
                    },
                ],
                "cores": [{"name": "cm7", "primary": True, "index": "0", "type": "cm7"}],
            },
            "debug": {
                "protocol": "swd",
                "connect-script": "RT1170_connect_M7.scp",
                "reset-script": "RT1170_reset_M7.scp",
                "swo": True,
            },
        }
    ],
}


def validate_bootloader(path):
    """Refuse anything that is not plausibly a bootloader for this part."""
    with open(path, 'rb') as f:
        img = f.read()

    if len(img) > BL_SLOT_SIZE:
        sys.exit("image is %d bytes - does not fit the %d byte bootloader slot"
                 % (len(img), BL_SLOT_SIZE))
    if len(img) < VECTORS_OFF + 8:
        sys.exit("image is too small (%d bytes) to be a bootloader" % len(img))

    tag = img[FCB_TAG_OFFSET:FCB_TAG_OFFSET + 4]
    if tag != FCB_TAG:
        sys.exit("no FCB tag %r at +0x%x (found %r) - this is not an RT117x "
                 "bootloader image" % (FCB_TAG, FCB_TAG_OFFSET, tag))

    msp, entry = struct.unpack('<II', img[VECTORS_OFF:VECTORS_OFF + 8])
    if msp in (0, 0xFFFFFFFF) or entry in (0, 0xFFFFFFFF):
        sys.exit("vector pair at +0x%x looks invalid" % VECTORS_OFF)
    if not (FLASH_BASE <= entry < FLASH_BASE + BL_SLOT_SIZE):
        sys.exit("entry 0x%08x is outside the bootloader slot - refusing"
                 % entry)

    print("image OK: %d bytes, FCB tag present, MSP=0x%08x entry=0x%08x"
          % (len(img), msp, entry))
    return len(img)


def linkserver(device_json, *args):
    cmd = [LINKSERVER, "flash", "--probe", PROBE_SERIAL, device_json] + list(args)
    return subprocess.run(cmd, capture_output=True, text=True)


def main():
    bl_image = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BL
    backup = (sys.argv[2] if len(sys.argv) > 2 else
              "bl_backup_%s.bin" % time.strftime("%Y%m%d_%H%M%S"))

    if not os.path.exists(LINKSERVER):
        sys.exit("LinkServer not found at %s" % LINKSERVER)
    if not os.path.exists(bl_image):
        sys.exit("no such bootloader image: %s" % bl_image)

    size = validate_bootloader(bl_image)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".json",
                                     delete=False) as f:
        json.dump(DEVICE_CONFIG, f)
        device_json = f.name

    print("backing up current bootloader region -> %s" % backup)
    r = linkserver(device_json, "dump", hex(FLASH_BASE), hex(BL_SLOT_SIZE), backup)
    if r.returncode != 0 or not os.path.exists(backup):
        sys.stdout.write(r.stdout)
        sys.stderr.write(r.stderr)
        sys.exit("backup FAILED - refusing to overwrite the only bootloader.\n"
                 "If detection failed ('no sectors' / '0B'), FlexSPI is in the "
                 "Zephyr-configured state and BOOT0 really is required.")
    print("  backed up %d bytes" % os.path.getsize(backup))

    load_arg = "%s:0x%08x" % (bl_image, FLASH_BASE)
    print("programming %s at 0x%08x (%d bytes, no mass erase)"
          % (bl_image, FLASH_BASE, size))
    r = linkserver(device_json, "load", load_arg, "-R")
    sys.stdout.write(r.stdout)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        sys.exit("programming FAILED - restore with:\n  %s %s"
                 % (sys.argv[0], backup))

    print("verifying")
    r = linkserver(device_json, "verify", load_arg)
    sys.stdout.write(r.stdout)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        sys.exit("VERIFY FAILED - restore with:\n  %s %s"
                 % (sys.argv[0], backup))

    print("\nAP_Bootloader installed and verified.")
    print("Reset the board:  python3 Tools/scripts/zephyr_pin_reset.py")
    print("Restore the old bootloader if ever needed:  %s %s"
          % (sys.argv[0], backup))
    return 0


if __name__ == '__main__':
    sys.exit(main())

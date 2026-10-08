#!/usr/bin/env python3
# encoding: utf-8

"""
Flash the mr_vmu_rt1176 ArduCopter build via LinkServer, using NXP's own
Octal-DDR flash driver (MIMXRT1170_SFDP_MXIC_OPI.cfx) over SWD.

AP_FLAKE8_CLEAN
"""

import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile


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
# Optional argv overrides:
#   rt1176_linkserver_flash.py <image.bin>          - flash to 0x30000000 with
#     a MASS ERASE first (-e). Used for bootloader installs. NOTE the mass
#     erase covers the whole 64MB and takes ~167s with no progress output -
#     do not wrap this script in short timeouts.
#   rt1176_linkserver_flash.py <image.bin> <0xADDR>  - STAGE the image at an
#     arbitrary flash address WITHOUT the mass erase (e.g. an MCUBoot A/B
#     slot-1 image at 0x30220000 = app region + 2MB). LinkServer erases the
#     sectors it programs; anything else in flash is left alone.
#
# The image argument is REQUIRED. It used to default to
# build/mr_vmu_rt1176/zephyr_upload.bin, so running this with no arguments at
# all mass-erased the whole 64MB - bootloader included - and then wrote the APP
# image at 0x30000000. That image is padded by APP_VECTOR_OFFSET for the
# bootloader, so its vector table landed at 0x30002000 while the app region
# starts at 0x30022000: a board with no bootloader AND an app that cannot
# start, recoverable only through BOOT0/SDP. Nothing about that is a sensible
# default, so there is no default.
if len(sys.argv) < 2:
    sys.exit(
        "usage: rt1176_linkserver_flash.py <image.bin> [0xADDR]\n"
        "  <image.bin>          MASS ERASES all 64MB, then writes at 0x30000000.\n"
        "                       This is the BOOTLOADER install path - pass a\n"
        "                       bootloader image, not build/.../zephyr_upload.bin.\n"
        "  <image.bin> <0xADDR> stages at that address with no mass erase.\n"
        "                       e.g. 0x30220000 for an MCUBoot slot-1 image."
    )

FIRMWARE = sys.argv[1]
FLASH_ADDR = sys.argv[2] if len(sys.argv) > 2 else "0x30000000"
MASS_ERASE = len(sys.argv) <= 2   # explicit address = staging, no mass erase

# Refuse the one combination that is always wrong: mass-erasing and then
# writing the padded APP image at the bootloader's address.
if MASS_ERASE and os.path.basename(FIRMWARE) == "zephyr_upload.bin":
    sys.exit(
        f"refusing: {FIRMWARE} is the padded APP image and this invocation "
        f"would mass-erase the bootloader and write it at {FLASH_ADDR}, "
        f"putting its vector table 0x2000 short of the app region.\n"
        f"To flash an app, use uploader.py, or pass an explicit address."
    )

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


def main():
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        json.dump(DEVICE_CONFIG, f)
        device_json = f.name

    load_arg = "%s:%s" % (FIRMWARE, FLASH_ADDR)
    probe_args = ["--probe", PROBE_SERIAL]
    load_cmd = [LINKSERVER, "flash"] + probe_args + [device_json, "load", load_arg]
    if MASS_ERASE:
        load_cmd.append("-e")
    load_cmd.append("-R")
    subprocess.run(load_cmd, check=True)
    subprocess.run([LINKSERVER, "flash"] + probe_args + [device_json, "verify", load_arg],
                   check=True)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode)

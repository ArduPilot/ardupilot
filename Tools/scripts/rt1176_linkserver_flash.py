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
FIRMWARE = sys.argv[1] if len(sys.argv) > 1 else "build/mr_vmu_rt1176/zephyr_upload.bin"
FLASH_ADDR = sys.argv[2] if len(sys.argv) > 2 else "0x30000000"
MASS_ERASE = len(sys.argv) <= 2   # explicit address = staging, no mass erase

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

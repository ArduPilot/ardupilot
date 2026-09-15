#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Flash a Zephyr binary directly to the RT1176's base flash address.

This is the flashing method that produced the first confirmed, stable
ArduPilot HAL boot on mr_vmu_rt1176 (2026-07-26): it writes straight to
0x30000000, bypassing the PX4 bootloader entirely. The app must be built
with CONFIG_NXP_IMXRT_BOOT_HEADER=y and CONFIG_FLASH_LOAD_OFFSET=0x0 (its
own FCB, no bootloader offset) - this is the *diagnostic* direct-boot
config, not the bootloader-relative production path.

Requires:
  - LinkServer installed (this script expects /usr/local/LinkServer_*/LinkServer;
    adjust LINKSERVER_BIN below if your install path differs).
  - Board power-cycled with BOOT0 held immediately before running this
    script - LinkServer's SWD flash driver only gets a clean connection
    right after a fresh BOOT0/ISP power-on. A normal boot (or a stale SWD
    session) will fail with "Wire ACK Fault" / "driver describes no
    sectors" - if you see that, power-cycle with BOOT0 held and re-run.

Usage:
  python3 Tools/scripts/rt1176_direct_flash.py [path/to/zephyr.bin]

  Defaults to build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.bin if no
  path is given.

After a successful flash, power-cycle the board *without* BOOT0 held to
boot the newly written image.
"""
import glob
import subprocess
import sys

from pathlib import Path

FLASH_ADDR = "0x30000000"
ERASE_SIZE = "0x40000"
DEVICE_JSON_CANDIDATES = [
    "Tools/scripts/rt1176_device.json",
    str(Path(__file__).parent / "rt1176_device.json"),
]
DEFAULT_BIN = "build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.bin"


def find_linkserver():
    matches = sorted(glob.glob("/usr/local/LinkServer_*/LinkServer"))
    if not matches:
        sys.exit("LinkServer not found under /usr/local/LinkServer_*/ - "
                 "edit LINKSERVER_BIN in this script if installed elsewhere.")
    return matches[-1]


def find_device_json():
    for candidate in DEVICE_JSON_CANDIDATES:
        if Path(candidate).exists():
            return candidate
    sys.exit("rt1176_device.json not found - see Tools/scripts/ for the "
             "expected LinkServer device config, or generate one with "
             "'LinkServer config' per the LinkServer docs.")


def run(cmd):
    print("+ " + " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True)
    print(result.stdout)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        sys.exit(f"Command failed (exit {result.returncode}). "
                 f"If this is a 'Wire ACK Fault' or 'no sectors' error, "
                 f"power-cycle the board with BOOT0 held and re-run.")
    return result.stdout


def main():
    bin_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BIN
    if not Path(bin_path).exists():
        sys.exit(f"Binary not found: {bin_path}")

    linkserver = find_linkserver()
    device_json = find_device_json()

    print(f"Flashing {bin_path} to {FLASH_ADDR} via {linkserver}")
    print("(board must be freshly power-cycled with BOOT0 held)\n")

    run([linkserver, "flash", device_json, "erase-range", FLASH_ADDR, ERASE_SIZE])
    run([linkserver, "flash", device_json, "load", "--addr", FLASH_ADDR, bin_path])
    run([linkserver, "flash", device_json, "verify", "--addr", FLASH_ADDR, bin_path])

    print("\nFlash write verified byte-exact.")
    print("Power-cycle the board WITHOUT BOOT0 held to boot the new image.")
    print("To watch boot output: python3 Tools/capture_uart.py --timeout 30")


if __name__ == "__main__":
    main()

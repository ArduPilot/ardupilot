#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Flash a Zephyr binary directly to RT1176 flash, WITHOUT touching the
installed PX4 v6xrt bootloader's own flash region.

BOOTLOADER-SAFE VARIANT of rt1176_direct_flash.py: that script's erase
range (0x30000000, 0x40000 bytes = 0x30000000-0x3003FFFF) covers the
entire installed bootloader (0x30000000-0x3001FFFF, 128KB reserved -
BOARD_FIRST_FLASH_SECTOR_TO_ERASE=32 sectors * 4KB in PX4's own
hw_config.h/main.c) - running it erases and destroys the bootloader,
requiring a full BOOT0/SWD reflash to recover (happened twice in one
session, 2026-07-27).

This variant targets APP_LOAD_ADDRESS (0x30020000) instead - the address
the PX4 bootloader itself uses as the base for app flash - and erases
only 0x20000 bytes (0x30020000-0x3003FFFF), never reaching back into the
bootloader's own 0x30000000-0x3001FFFF region. Use this for direct-SWD
boot testing that must coexist with a bootloader you don't want to
destroy. The app must still be built for direct/no-offset execution at
this address (CONFIG_NXP_IMXRT_BOOT_HEADER=y, CONFIG_FLASH_LOAD_OFFSET=0x0
relative to APP_LOAD_ADDRESS - i.e. linked to run starting at 0x30020000,
NOT 0x30000000; this differs from the original script, which targeted
true flash base 0x30000000 with no bootloader present at all).

Requires:
  - LinkServer installed (this script expects /usr/local/LinkServer_*/LinkServer;
    adjust LINKSERVER_BIN below if your install path differs).
  - Board power-cycled with BOOT0 held immediately before running this
    script - LinkServer's SWD flash driver only gets a clean connection
    right after a fresh BOOT0/ISP power-on. A normal boot (or a stale SWD
    session) will fail with "Wire ACK Fault" / "driver describes no
    sectors" - if you see that, power-cycle with BOOT0 held and re-run.

Usage:
  python3 Tools/scripts/rt1176_direct_flash_dont_destroy_bootloader.py [path/to/zephyr.bin]

  Defaults to build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.bin if no
  path is given.

After a successful flash, power-cycle the board *without* BOOT0 held to
boot the newly written image.
"""
import glob
import subprocess
import sys

from pathlib import Path

FLASH_ADDR = "0x30020000"  # APP_LOAD_ADDRESS - never touches the bootloader
ERASE_SIZE = "0x20000"     # stops well short of 0x30000000-0x3001FFFF

# APP_VECTOR_OFFSET (PX4 v6xrt's own hw_config.h constant) - a correctly
# padded .bin (see Tools/ardupilotwaf/zephyr.py's
# _pad_bin_for_flash_load_offset(), used by uploader.py's upload path) has
# this many bytes of 0xFF before the real vector table. This script writes
# to APP_LOAD_ADDRESS just like uploader.py does, so it needs the SAME
# padded file uploader.py uses - flashing the unpadded zephyr.bin here
# would silently reproduce the exact vector-table-offset bug that .bin
# padding was added to fix (2026-07-27).
APP_VECTOR_OFFSET = 0x2000
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


def check_bin_is_padded(bin_path):
    """Warn (don't block - the check can have false positives) if bin_path
    doesn't look like it has APP_VECTOR_OFFSET bytes of 0xFF padding before
    a real vector table. See APP_VECTOR_OFFSET comment above."""
    data = Path(bin_path).read_bytes()

    if len(data) <= APP_VECTOR_OFFSET + 4:
        print(f"[!] WARNING: {bin_path} is only {len(data)} bytes - too "
              f"small to contain {APP_VECTOR_OFFSET}-byte padding plus a "
              f"vector table. This looks like an UNPADDED .bin (e.g. "
              f"build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.bin) - "
              f"flashing it to {FLASH_ADDR} will very likely reproduce the "
              f"2026-07-27 vector-table-offset bug. Use the PADDED file "
              f"instead (build/mr_vmu_rt1176/zephyr_upload.bin, the same "
              f"one uploader.py uses).")
        return

    leading = data[:APP_VECTOR_OFFSET]
    vt_word = int.from_bytes(data[APP_VECTOR_OFFSET:APP_VECTOR_OFFSET + 4],
                             byteorder="little")
    looks_padded = (leading == b"\xFF" * APP_VECTOR_OFFSET) and (vt_word & 0x20000000)

    if not looks_padded:
        print(f"[!] WARNING: {bin_path} does not look like it has the "
              f"expected {APP_VECTOR_OFFSET}-byte 0xFF padding before a "
              f"valid vector table (checked byte offset 0x{APP_VECTOR_OFFSET:x}). "
              f"This looks like an UNPADDED .bin (e.g. "
              f"build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.bin) - "
              f"flashing it to {FLASH_ADDR} will very likely reproduce the "
              f"2026-07-27 vector-table-offset bug (app vector table lands "
              f"{APP_VECTOR_OFFSET} bytes too early, jump_to_app()-equivalent "
              f"checks fail). Use the PADDED file instead "
              f"(build/mr_vmu_rt1176/zephyr_upload.bin, the same one "
              f"uploader.py uses).")


def main():
    bin_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BIN
    if not Path(bin_path).exists():
        sys.exit(f"Binary not found: {bin_path}")

    check_bin_is_padded(bin_path)

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

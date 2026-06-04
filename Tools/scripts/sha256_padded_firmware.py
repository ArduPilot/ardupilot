#!/usr/bin/env python3
"""
Compute the SHA-256 of a firmware .bin file padded to the size of the
application flash region, matching the hash produced by the ArduPilot
checksum_params_and_fw.lua / checksum_firmware.lua scripting examples.

Background
----------
On boards such as the Pixhawk6X (STM32H7) the internal flash is divided
into two regions:

  [ bootloader region ][ application region ]
  0x08000000          0x08000000 + FLASH_RESERVE_START_KB * 1024

The bootloader lives in the first FLASH_RESERVE_START_KB kilobytes (128 KB
on most H7-based boards).  The compiled application binary (.bin) is flashed
immediately after it.

When ArduPilot reads @SYS/flash.bin it exposes the *entire* flash starting
at 0x08000000 for FLASH_SIZE_KB kilobytes.  The Lua scripting examples skip
the bootloader region with fs:sha256("@SYS/flash.bin", RESERVE_KB*1024) and
then hash the remaining (FLASH_SIZE_KB - FLASH_RESERVE_START_KB)*1024 bytes.

The application binary is almost always shorter than the application region.
Flash cells that have never been written (or were erased) read back as 0xFF,
so the autopilot sees [firmware bytes][0xFF ... 0xFF] to fill the region.

Therefore, to compute the same digest offline you must:
  1. Read the raw .bin file produced by the build.
  2. Append 0xFF bytes until the data is exactly
     (FLASH_SIZE_KB - FLASH_RESERVE_START_KB) * 1024 bytes long.
  3. SHA-256 the result.

The defaults (flash=2048 KB, reserve=128 KB → 1920 KB region) match the
Pixhawk6X / STM32H7 family.  Pass explicit values for other boards; consult
the board's hwdef.dat for FLASH_SIZE_KB and FLASH_RESERVE_START_KB.

Usage
-----
  sha256_padded_firmware.py <firmware.bin> [flash_kb [reserve_kb]]

Examples
--------
  # Pixhawk6X (defaults)
  sha256_padded_firmware.py build/Pixhawk6X/bin/arducopter.bin

  # Board with 1024 KB flash and 64 KB bootloader region
  sha256_padded_firmware.py build/MyBoard/bin/arducopter.bin 1024 64
"""

import hashlib
import sys


def main():
    path       = sys.argv[1] if len(sys.argv) > 1 else "build/Pixhawk6X/bin/arducopter.bin"
    flash_kb   = int(sys.argv[2]) if len(sys.argv) > 2 else 2048
    reserve_kb = int(sys.argv[3]) if len(sys.argv) > 3 else 128

    region_size = (flash_kb - reserve_kb) * 1024

    with open(path, "rb") as f:
        data = f.read()

    if len(data) > region_size:
        print(f"WARNING: firmware ({len(data)} B) is larger than the "
              f"application region ({region_size} B) — hash will be wrong")

    # Pad with 0xFF to match erased flash cells beyond the end of the binary.
    padded = data + b"\xff" * max(0, region_size - len(data))
    digest = hashlib.sha256(padded).hexdigest().upper()

    print(f"file:        {path}")
    print(f"file size:   {len(data)} B  ({len(data)/1024:.1f} KB)")
    print(f"region size: {region_size} B  ({region_size//1024} KB)  "
          f"[{flash_kb} KB flash - {reserve_kb} KB bootloader]")
    print(f"SHA-256:     {digest}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Upload an MCUBoot-format image to a running ArduPilot over the mcumgr SMP
protocol (USB CDC, second ACM interface = the zephyr,uart-mcumgr chosen),
then optionally reset so AP_Bootloader's MCUBoot-compatible A/B installs it.

This is the standard mcumgr/smpclient ecosystem flow: the SMP server runs in
the APPLICATION (CONFIG_MCUMGR + img_mgmt writes slot1_partition through the
ROM-API flash facade); the bootloader only validates+installs at boot.

Usage:
  zephyr_smp_upload.py <image.img> <port> [--reset]
  e.g.
  zephyr_smp_upload.py build/mr_vmu_rt1176/ap_firmware_mr_vmu_rt1176.img \
      /dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176_..._-if02 --reset

Requires: pip install smpclient
"""

import asyncio
import sys
import time

from smpclient import SMPClient
from smpclient.requests.os_management import EchoWrite
from smpclient.requests.os_management import ResetWrite
from smpclient.transport.serial import SMPSerialTransport


async def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    image_path = sys.argv[1]
    port = sys.argv[2]
    do_reset = '--reset' in sys.argv[3:]

    with open(image_path, 'rb') as f:
        image = f.read()
    print("image: %s (%u bytes)" % (image_path, len(image)))

    async with SMPClient(SMPSerialTransport(), port) as client:
        r = await client.request(EchoWrite(d="ap"))
        print("echo response: %r" % getattr(r, 'r', r))

        t0 = time.time()
        done = 0
        async for offset in client.upload(image, slot=1):
            done = offset
            pct = 100.0 * offset / len(image)
            print("\rupload: %6.2f%% (%u/%u)" % (pct, offset, len(image)),
                  end='', flush=True)
        dt = time.time() - t0
        print("\nuploaded %u bytes in %.1fs (%.1f kB/s)" %
              (done, dt, done / dt / 1024.0 if dt > 0 else 0))

        if do_reset:
            print("sending reset (A/B install runs in the bootloader)...")
            await client.request(ResetWrite())
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))

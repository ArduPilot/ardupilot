#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Race to open the USB CDC console so the FIRST boot after a flash ( or a regular reboot) is captured.

The board's CDC port appears the instant it enumerates. If you poll for the
device node and open it within a fraction of a second, you get the boot output
from the banner onward - "Init ArduCopter", sensor probes, "Successfully
mounted SDCard", and so on. Open it late and all of that is already gone,
because nothing buffers it.

This tool, by-design tries to ignore or filter mavlink binary.

That gives a readiness signal in a couple of SECONDS rather than a blind
90-second sleep, and it is the only reliable way to read this board's boot
messages: the LPUART1 console goes through the MCU-Link bridge, which routinely
delivers zero bytes, whereas MAVLink and printf both reach the USB CDC.

Run it BEFORE resetting or flashing the board, so it is already waiting when
the port appears:

    Tools/scripts/zephyr_bootlog.py --out boot.log --secs 40 &
    Tools/scripts/zephyr_flash.sh

Exits 0 if anything was captured, 1 if the port never appeared.

⚠ Only ONE reader may hold the CDC port. Do not run this alongside a pymavlink
session or a second capture - they starve each other and look like a dead board.
"""
import argparse
import glob
import sys
import time

# The interface suffix is NOT stable across boards or firmware revisions. A
# single-interface CDC enumerates as '-if00', but a composite device names its
# interfaces: mr_vmu_rt1176 running the MAVLink + SMP pair appears as
# '-if-mavlink' and '-if-smp', and nothing matches '-if00' at all. Globbing for
# '-if00' alone made this tool sit out its whole --wait and report "app CDC port
# never appeared" on a board that was up and talking. Match the console
# interface of either shape, and let --port override when neither fits.
BY_ID = ['/dev/serial/by-id/usb-ArduPilot*-if00',
         '/dev/serial/by-id/usb-ArduPilot*-if-mavlink']
BL_ID = ['/dev/serial/by-id/usb-ArduPilot*-BL_*-if00',
         '/dev/serial/by-id/usb-ArduPilot*-BL_*-if-mavlink']
POLL = 0.02          # 20 ms - the open must win the race against the banner
BANNER_HINTS = ('ArduCopter', 'Init', 'ArduPilot', 'mounted SDCard')


def find_port(patterns):
    for pat in patterns:
        m = sorted(glob.glob(pat))
        if m:
            return m[0]
    return None


def main():
    print('[pls connect your USB device now, and wait %.0f seconds for the boot log to be captured]' % 10.0)
    ap = argparse.ArgumentParser()
    # yyyy-mm-dd-hh-mm-ss
    datetime_as_string = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
    ap.add_argument('--out', default=f'boot_{datetime_as_string}.log')
    ap.add_argument('--secs', type=float, default=10.0, help='capture window once open')
    ap.add_argument('--wait', type=float, default=120.0, help='how long to wait for the port')
    ap.add_argument('--port', default=None,
                    help='/dev/serial/by-id/ path or glob to capture, instead of the '
                         'built-in ArduPilot console patterns')
    args = ap.parse_args()

    import serial

    # Wait for the APP port. The bootloader enumerates under a '-BL' name; seeing
    # that is normal mid-flash and simply means the app has not started yet.
    patterns = [args.port] if args.port else BY_ID
    deadline = time.time() + args.wait
    port = None
    while time.time() < deadline:
        port = find_port(patterns)
        if port:
            break
        time.sleep(POLL)
    if not port:
        bl = any(glob.glob(pat) for pat in BL_ID)
        print('bootlog: app CDC port never appeared within %.0fs (bootloader present: %s)'
              % (args.wait, bl))
        print('bootlog: patterns tried: %s' % ', '.join(patterns))
        return 1

    # Open IMMEDIATELY. udev may still be settling permissions, so retry hard
    # for a moment rather than giving up on the first EACCES/ENOENT.
    ser = None
    open_deadline = time.time() + 5.0
    while time.time() < open_deadline:
        try:
            ser = serial.Serial(port, 115200, timeout=0.1)
            break
        except Exception:  # noqa: BLE001
            time.sleep(POLL)
    if ser is None:
        print('bootlog: port %s appeared but could not be opened' % port)
        return 1

    t_open = time.time()
    # print('bootlog: opened %s' % os.path.basename(port))

    buf = bytearray()
    first_hint = None
    # A reset taken while this tool is already attached tears the USB CDC device
    # down under it: the read raises, and a capture that simply stopped there
    # returned the handful of bytes from BEFORE the reset and reported success.
    # That is the wrong half of the run. Reopen across the gap instead, until
    # the capture window is genuinely spent.
    while time.time() - t_open < args.secs:
        try:
            d = ser.read(4096)
        except Exception:  # noqa: BLE001
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
            ser = None
            while ser is None and time.time() - t_open < args.secs:
                port = find_port(patterns)
                if port:
                    try:
                        ser = serial.Serial(port, 115200, timeout=0.1)
                    except Exception:  # noqa: BLE001
                        ser = None
                time.sleep(POLL)
            if ser is None:
                break
            continue
        if d:
            buf += d
            if first_hint is None:
                txt = buf.decode('utf-8', 'replace')
                for h in BANNER_HINTS:
                    if h in txt:
                        first_hint = (h, time.time() - t_open)
                        # print('bootlog: saw %r at +%.1fs' % (h, first_hint[1]))
                        break
    if ser is not None:
        ser.close()

    text = buf.decode('utf-8', 'replace')
    with open(args.out, 'w') as f:
        # f.write('# USB CDC boot capture, port opened %.2fs after it appeared\n' % 0.0)
        # f.write('# %d bytes over %.1fs\n\n' % (len(buf), args.secs))
        f.write(text)
    # print('bootlog: %d bytes -> %s' % (len(buf), args.out))

    # The stream carries MAVLink alongside text; that is expected and is
    # positive evidence the telemetry stack is live, so do not filter it out.
    printable = ''.join(c for c in text if 32 <= ord(c) < 127 or c in '\r\n')
    for line in printable.splitlines():
        if any(h in line for h in BANNER_HINTS) or 'SDCard' in line or 'Init' in line:
            print('   | %s' % line.strip()[:100])

    # The capture itself to stdout: no stats, no header, no footer, no length,
    # just what the board said. Written from here rather than after main()
    # returns, because the filename is built from a timestamp local to this
    # function - reconstructing it outside got the wrong name, and ignored
    # --out entirely.
    sys.stdout.write(text)

    return 0 if buf else 1


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# encoding: utf-8

"""
Flash SiK firmware to an RFD900x radio over a plain serial port.

Protocol follows RFDesign's own txmod implementation
(github.com/RFDesign/mavesp8266, branch txmod: lib/XModem/XModem.cpp and
src/main.cpp), which is the reference for how these radios are updated:

  1. Enter AT command mode: "+++" with a 1 s guard either side. If the radio
     is already in its bootloader it answers with one of the hex signatures
     below instead, and command mode is skipped.
  2. "ATI\\r" - expect "SiK" in the reply (the running firmware), or the
     bootloader signatures C1 E4 E3 F8 / C1 E4 E7 F8 meaning we are already
     in the bootloader.
  3. "\\r\\n", 200 ms, then "AT&UPDATE\\r" - CR ONLY, a trailing \\n breaks it -
     then 700 ms for the radio to drop into its bootloader.
  4. Write "U" repeatedly as the bootloader's autobauder and wait for it to
     announce itself with "ChipID:" or "UPLOAD".
  5. XModem/XModem-1K send of the .bin: the receiver sends 'C' for CRC mode
     (a following 'K' selects 1024-byte blocks) or NAK for the classic
     checksum. Blocks are SOH/STX, seq, ~seq, payload, then CRC16-CCITT
     (poly 0x1021, init 0, big-endian) or the inverse-sum checksum. Each
     block is ACKed; EOT ends the transfer.

Firmware images come from https://files.rfdesign.com.au/firmware/ - use the
"RFDSiK V<x.yy> rfd900x.bin" files for RFD900X hardware. The rfd900x2 /
.gbl images are for the newer x2 boards and will NOT work here.

USAGE
    rfd900x_flash.py <port> <firmware.bin> [--baud N]

    rfd900x_flash.py /dev/ttyUSB0 "RFDSiK V3.57 rfd900x.bin"

The radio must be on a direct serial link (an FTDI lead), not on the far end
of a radio hop - the bootloader is not reachable over the air.

AP_FLAKE8_CLEAN
"""

import argparse
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial required: pip install pyserial")

SOH, STX, EOT, ACK, NAK, CAN = 0x01, 0x02, 0x04, 0x06, 0x15, 0x18
BOOTLOADER_SIGS = (b'\xC1\xE4\xE3\xF8', b'\xC1\xE4\xE7\xF8')
# baud rates to try, stock first (main.cpp's baud_list)
BAUD_LIST = [57600, 115200, 9600, 19200, 38400, 230400, 460800]


def crc16_ccitt(data, crc=0):
    """CRC16-CCITT exactly as XModem.cpp::outputByte accumulates it."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def read_for(port, seconds):
    end = time.time() + seconds
    buf = b''
    while time.time() < end:
        buf += port.read(256)
    return buf


def enter_command_mode(port):
    """Return 'cmd', 'bootloader', or None."""
    port.reset_input_buffer()
    port.write(b'ATI\r')
    port.flush()
    r = read_for(port, 1.5)
    if any(sig in r for sig in BOOTLOADER_SIGS):
        return 'bootloader'
    if b'SiK' in r or b'RFD' in r:
        return 'cmd'

    # not talking yet - do the +++ dance with guard times
    port.reset_input_buffer()
    time.sleep(1.2)
    port.write(b'+++')
    port.flush()
    time.sleep(1.5)
    read_for(port, 0.3)
    port.write(b'ATI\r')
    port.flush()
    r = read_for(port, 1.5)
    if any(sig in r for sig in BOOTLOADER_SIGS):
        return 'bootloader'
    if b'SiK' in r or b'RFD' in r:
        return 'cmd'
    return None


def request_update(port):
    """Send AT&UPDATE. CR only - a trailing newline breaks it."""
    port.write(b'\r\n')
    port.flush()
    time.sleep(0.2)
    port.reset_input_buffer()
    port.write(b'AT&UPDATE\r')
    port.flush()
    time.sleep(0.7)


def bootloader_sync(port, tries=30):
    """Autobaud the bootloader with 'U' until it announces itself."""
    for _ in range(tries):
        port.write(b'U')
        port.flush()
        r = read_for(port, 0.5)
        if b'ChipID:' in r or b'UPLOAD' in r:
            port.reset_input_buffer()
            return True
    return False


def start_upload(port):
    """Send UPLOAD and wait for 'Ready' - the bootloader will not begin the
    XModem handshake (no 'C'/NAK) until this command is acknowledged."""
    port.reset_input_buffer()
    port.write(b'UPLOAD\r')
    port.flush()
    end = time.time() + 3
    buf = b''
    while time.time() < end:
        buf += port.read(256)
        if b'Ready' in buf:
            time.sleep(0.2)
            read_for(port, 0.2)      # swallow the trailing \r\n
            return True
    return False


def boot_new(port):
    """Tell the bootloader to run the freshly written firmware."""
    port.write(b'BOOTNEW\r')
    port.flush()
    time.sleep(0.5)


def xmodem_send(port, data, progress=True):
    """XModem/1K send. Returns True on success."""
    # wait for the receiver to announce its mode
    packet_len, use_crc = 128, True
    deadline = time.time() + 30
    while True:
        if time.time() > deadline:
            print("  no XModem start character from bootloader")
            return False
        c = port.read(1)
        if not c:
            continue
        if c[0] == NAK:
            use_crc = False
            break
        if c == b'C':
            use_crc = True
            # a following 'K' means it accepts 1024-byte blocks
            nxt = port.read(1)
            if nxt == b'K':
                packet_len = 1024
            break

    print("  XModem: %d-byte blocks, %s"
          % (packet_len, "CRC16" if use_crc else "checksum"))

    total = (len(data) + packet_len - 1) // packet_len
    seq = 1
    for i in range(total):
        chunk = data[i * packet_len:(i + 1) * packet_len]
        chunk = chunk + b'\x00' * (packet_len - len(chunk))   # pad with zeroes
        header = bytes([SOH if packet_len == 128 else STX, seq & 0xFF,
                        (~seq) & 0xFF])
        if use_crc:
            crc = crc16_ccitt(chunk)
            tail = bytes([(crc >> 8) & 0xFF, crc & 0xFF])
        else:
            tail = bytes([(255 - (sum(chunk) & 0xFF)) & 0xFF])

        for attempt in range(30):
            port.write(header + chunk + tail)
            port.flush()
            resp = b''
            end = time.time() + 4
            while time.time() < end and not resp:
                resp = port.read(1)
            if resp and resp[0] == ACK:
                break
            if resp and resp[0] == CAN:
                print("  receiver cancelled")
                return False
        else:
            print("  block %d failed after 30 retries" % seq)
            return False

        seq = (seq + 1) & 0xFF
        if progress and (i % 20 == 0 or i == total - 1):
            pct = 100.0 * (i + 1) / total
            print("\r  programming: %5.1f%% (%d/%d blocks)" % (pct, i + 1, total),
                  end='', flush=True)
    print()

    for _ in range(10):
        port.write(bytes([EOT]))
        port.flush()
        r = port.read(1)
        if r and r[0] == ACK:
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('port')
    ap.add_argument('firmware')
    ap.add_argument('--baud', type=int, default=None,
                    help='force a baud rate instead of searching')
    args = ap.parse_args()

    if not os.path.exists(args.firmware):
        sys.exit("no such firmware file: %s" % args.firmware)
    with open(args.firmware, 'rb') as f:
        fw = f.read()
    print("firmware: %s (%d bytes)" % (os.path.basename(args.firmware), len(fw)))
    # sanity: these images start with an ARM vector table (initial SP in RAM)
    sp = int.from_bytes(fw[0:4], 'little')
    if not (0x20000000 <= sp <= 0x20040000):
        sys.exit("does not look like RFD900x firmware (initial SP 0x%08x)" % sp)

    bauds = [args.baud] if args.baud else BAUD_LIST
    for baud in bauds:
        print("trying %d baud ..." % baud)
        try:
            port = serial.Serial(args.port, baud, timeout=0.3)
        except Exception as e:                       # noqa: BLE001
            sys.exit("cannot open %s: %s" % (args.port, e))
        try:
            state = enter_command_mode(port)
            # No answer does NOT mean no radio: a unit left in its bootloader
            # by a previous attempt ignores ATI entirely. Carry on and let the
            # bootloader sync decide, exactly as txmod's main.cpp does.
            print("  radio responded (%s)" % (state or "silent - trying bootloader anyway"))
            if state == 'cmd':
                request_update(port)
            if not bootloader_sync(port):
                print("  bootloader did not sync")
                port.close()
                continue
            if not start_upload(port):
                print("  bootloader did not answer UPLOAD with 'Ready'")
                port.close()
                continue
            print("  bootloader ready - starting transfer")
            ok = xmodem_send(port, fw)
            if ok:
                boot_new(port)
            port.close()
            if ok:
                print("\nFIRMWARE UPDATE COMPLETE - new firmware booted")
                return 0
            print("\nTRANSFER FAILED - the radio is still in its bootloader;")
            print("re-run this tool to retry (it will resync automatically)")
            return 1
        except Exception as e:                       # noqa: BLE001
            print("  error: %s" % e)
            try:
                port.close()
            except Exception:                        # noqa: BLE001
                pass
    sys.exit("no radio found on %s" % args.port)


if __name__ == '__main__':
    sys.exit(main())

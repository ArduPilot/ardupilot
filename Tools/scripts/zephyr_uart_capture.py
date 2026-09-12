#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Serial console capture to a timestamped evidence file.

Captures everything a serial port emits into ZEPHYR_BOOT_LOG_<timestamp>.txt
(or --output). Keep the naming convention so captures stay recognisable, and
treat them as immutable evidence - commit them, never edit them.

ALWAYS pass --port with a /dev/serial/by-id/ path: two CDC devices are
normally present on this bench (RT1176 app + debug probe, sometimes a
CubeOrange too) and a bare /dev/ttyACM<n> number silently talks to the WRONG
one after any re-enumeration. There is deliberately no ttyACM default.

For racing the FIRST boot after a flash on the RT1176's USB CDC, prefer
Tools/scripts/zephyr_bootlog.py, which retries the open until the device
enumerates. This tool is the generic capture: any port, any duration,
timestamped file per run.

Usage:
    python3 Tools/scripts/zephyr_uart_capture.py --port /dev/serial/by-id/<dev>
    python3 Tools/scripts/zephyr_uart_capture.py --port ... --baud 115200 --timeout 120
"""

import argparse
import sys
import time

from datetime import datetime
from pathlib import Path

import serial


def capture_uart(port, baudrate=115200, output=None, timeout=None):
    """
    Capture UART output to file and console.

    Args:
        port: Serial port path
        baudrate: Baud rate
        output: Output file path (default: ZEPHYR_BOOT_LOG_<timestamp>.txt)
        timeout: Timeout in seconds (None = run until Ctrl+C)
    """

    # Generate output filename if not specified
    if output is None:
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        output = f'ZEPHYR_BOOT_LOG_{timestamp}.txt'

    output_path = Path(output)
    print("UART Capture Tool")
    print("=" * 70)
    print(f"Port:     {port}")
    print(f"Baud:     {baudrate}")
    print(f"Output:   {output_path.absolute()}")
    print(f"Timeout:  {timeout}s" if timeout else "Timeout:  None (Ctrl+C to stop)")
    print("=" * 70)
    print()

    try:
        # Open serial port
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"[✓] Connected to {port} @ {baudrate} baud")
        print("[•] Waiting for output...")
        print()

        # Open output file in BINARY mode.
        #
        # This port carries MAVLink as well as text, so decoding to str before
        # writing destroys every non-UTF-8 byte (errors='replace' turns each
        # one into U+FFFD). That had two bad effects: the MAVLink was
        # unrecoverable, so it could not be decoded as an independent check on
        # what the firmware was doing when console markers were unreliable; and
        # U+FFFD is three bytes, so a MAVLink-heavy capture LOOKED larger while
        # actually carrying less console output, making file size a misleading
        # proxy for how much was logged.
        #
        # Raw bytes stay greppable with `grep -a`, and can now be fed to a
        # MAVLink parser.
        with open(output_path, 'wb') as f:
            f.write(b"UART Capture Log\n")
            f.write(f"Port: {port} @ {baudrate} baud\n".encode())
            f.write(f"Started: {datetime.now().isoformat()}\n".encode())
            f.write(b"=" * 70 + b"\n\n")

            start_time = time.time()
            bytes_captured = 0

            while True:
                # Check timeout
                if timeout and (time.time() - start_time) > timeout:
                    print(f"\n[✓] Timeout reached ({timeout}s)")
                    break

                # Try to read from serial
                try:
                    if ser.in_waiting > 0:
                        data = ser.read(min(1024, ser.in_waiting))

                        # Console gets a lossy text rendering for readability;
                        # the FILE gets the raw bytes, so nothing is lost.
                        sys.stdout.write(data.decode('utf-8', errors='replace'))
                        sys.stdout.flush()

                        f.write(data)
                        f.flush()

                        bytes_captured += len(data)
                except Exception as e:  # noqa: BLE001
                    print(f"[!] Error reading: {e}")
                    break

                time.sleep(0.01)

        # Summary
        elapsed = time.time() - start_time
        print()
        print("=" * 70)
        print("[✓] Capture complete")
        print(f"    Duration: {elapsed:.1f} seconds")
        print(f"    Bytes captured: {bytes_captured}")
        print(f"    Output: {output_path.absolute()}")

        ser.close()
        return True

    except FileNotFoundError:
        print(f"[✗] Port {port} not found")
        print("    Available ports:")
        import glob
        for p in glob.glob('/dev/tty*'):
            print(f"      {p}")
        return False

    except serial.SerialException as e:
        print(f"[✗] Serial error: {e}")
        return False

    except KeyboardInterrupt:
        print("\n[•] Interrupted by user")
        elapsed = time.time() - start_time
        print(f"    Captured {bytes_captured} bytes in {elapsed:.1f}s")
        print(f"    Output: {output_path.absolute()}")
        return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Capture RT1176 UART console output during Zephyr boot'
    )
    parser.add_argument('--port', required=True,
                        help='Serial port - ALWAYS a /dev/serial/by-id/ path (see module docstring)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--output', default=None,
                        help='Output file (default: ZEPHYR_BOOT_LOG_<timestamp>.txt)')
    parser.add_argument('--timeout', type=int, default=20,
                        help='Timeout in seconds (default: 20). A full '
                             'mr_vmu_rt1176 boot needs ~20s to reach '
                             'ArduPilot gyro calibration: the bootloader '
                             'holds for 5s, then CONFIG_AP_SPI_PROBE_DIAG '
                             'and CONFIG_AP_I2C_PROBE_DIAG print their '
                             'scans before INS init. 10s cut off mid-boot.')

    args = parser.parse_args()

    success = capture_uart(
        port=args.port,
        baudrate=args.baud,
        output=args.output,
        timeout=args.timeout
    )

    sys.exit(0 if success else 1)

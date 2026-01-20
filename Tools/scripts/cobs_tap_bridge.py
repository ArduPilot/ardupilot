#!/usr/bin/env python3
"""
COBS TAP Bridge - Bridge a TAP interface to serial using COBS encoding

This allows a Linux machine to participate in an ArduPilot COBS Ethernet network.
Use with ArduPilot's COBS-over-serial Ethernet hub feature.

Usage:
    sudo ./cobs_tap_bridge.py /dev/ttyUSB0 --ip 192.168.13.100/24

Frame format (matching ArduPilot):
    - Ethernet frame + 4-byte CRC32 (little-endian)
    - COBS encoded
    - 0x00 byte delimiter between frames

Author: ArduPilot
License: GPLv3
"""

import argparse
import fcntl
import os
import select
import struct
import subprocess
import sys
import time

# For serial port
try:
    import serial
except ImportError:
    print("Please install pyserial: pip install pyserial")
    sys.exit(1)

# Linux TAP interface constants
TUNSETIFF = 0x400454ca
IFF_TAP = 0x0002
IFF_NO_PI = 0x1000

# CRC32 table (same as ArduPilot's crc32_tab)
_CRC32_TAB = [
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
    0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
    0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
    0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
    0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
    0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
    0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
    0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
    0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
    0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
    0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
    0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
    0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
    0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
    0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
    0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
    0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
    0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
    0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
    0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
    0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
    0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
    0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
    0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
    0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
    0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
    0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
    0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
]


def crc32(data):
    """Calculate CRC32 matching ArduPilot's crc_crc32(0, data, len)"""
    crc = 0
    for byte in data:
        crc = _CRC32_TAB[(crc ^ byte) & 0xff] ^ (crc >> 8)
    return crc


def cobs_encode(data):
    """COBS encode data, returns encoded bytes (without delimiter)"""
    output = bytearray()
    code_idx = 0
    output.append(0)  # Placeholder for first code byte
    code = 1

    for byte in data:
        if byte == 0:
            output[code_idx] = code
            code_idx = len(output)
            output.append(0)  # Placeholder for next code byte
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_idx] = code
                code_idx = len(output)
                output.append(0)  # Placeholder for next code byte
                code = 1

    output[code_idx] = code
    return bytes(output)


def cobs_decode(data):
    """COBS decode data, returns decoded bytes or None on error"""
    if len(data) == 0:
        return None

    output = bytearray()
    idx = 0

    while idx < len(data):
        code = data[idx]
        if code == 0:
            return None  # Invalid: zero in COBS data
        idx += 1

        for _ in range(code - 1):
            if idx >= len(data):
                return None  # Truncated
            output.append(data[idx])
            idx += 1

        if code < 0xFF and idx < len(data):
            output.append(0)

    return bytes(output)


KEEPALIVE_PAYLOAD = b'KALIVE\x01'


class COBSTapBridge:
    def __init__(self, serial_port, baud_rate, tap_name='tap0', ip_addr=None,
                 rtscts=False, verbose=False):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.tap_name = tap_name
        self.ip_addr = ip_addr
        self.rtscts = rtscts
        self.verbose = verbose
        self.running = False
        self.tap_fd = None
        self.ser = None

        # Stats
        self.rx_frames = 0
        self.tx_frames = 0
        self.rx_errors = 0
        self.crc_errors = 0
        self.keepalives_rx = 0
        self.keepalives_tx = 0

        # RX buffer for accumulating COBS frames
        self.rx_buffer = bytearray()

    def open_tap(self):
        """Create and open TAP interface"""
        tap_fd = os.open('/dev/net/tun', os.O_RDWR)

        # Configure as TAP (Ethernet) interface
        ifr = struct.pack('16sH', self.tap_name.encode(), IFF_TAP | IFF_NO_PI)
        fcntl.ioctl(tap_fd, TUNSETIFF, ifr)

        self.tap_fd = tap_fd
        print(f"Opened TAP interface: {self.tap_name}")
        return tap_fd

    def configure_tap(self):
        """Configure TAP interface with IP address"""
        if not self.ip_addr:
            print(f"Configure interface: sudo ip addr add <IP>/<MASK> dev {self.tap_name} "
                  f"&& sudo ip link set {self.tap_name} up")
            return

        try:
            subprocess.run(['ip', 'addr', 'add', self.ip_addr, 'dev', self.tap_name], check=True)
            subprocess.run(['ip', 'link', 'set', self.tap_name, 'up'], check=True)
            print(f"Configured {self.tap_name} with {self.ip_addr}")
        except subprocess.CalledProcessError as e:
            print(f"Warning: Failed to configure interface: {e}")

    def open_serial(self):
        """Open serial port"""
        self.ser = serial.Serial(
            self.serial_port,
            self.baud_rate,
            timeout=0.01,
            write_timeout=1.0,
            rtscts=self.rtscts,
            exclusive=True
        )
        # Flush any stale data
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"Opened serial port: {self.serial_port} at {self.baud_rate} baud")
        return self.ser

    def send_frame_to_serial(self, frame):
        """Send Ethernet frame over serial with COBS encoding"""
        # Append CRC32 (little-endian)
        crc = crc32(frame)
        frame_with_crc = frame + struct.pack('<I', crc)

        # COBS encode
        encoded = cobs_encode(frame_with_crc)

        # Send with delimiter
        packet = encoded + b'\x00'
        self.ser.write(packet)
        self.ser.flush()  # Ensure data is sent
        self.tx_frames += 1

        if self.verbose:
            print(f"TX: {len(frame)} bytes, CRC={crc:08x} -> {len(packet)} encoded")

    def send_keepalive(self):
        """Send keepalive frame"""
        crc = crc32(KEEPALIVE_PAYLOAD)
        frame_with_crc = KEEPALIVE_PAYLOAD + struct.pack('<I', crc)
        encoded = cobs_encode(frame_with_crc)
        self.ser.write(encoded + b'\x00')
        self.keepalives_tx += 1

    def process_serial_rx(self):
        """Read from serial and process COBS frames"""
        data = self.ser.read(4096)
        if not data:
            return

        self.rx_buffer.extend(data)

        # Process complete frames (delimited by 0x00)
        while b'\x00' in self.rx_buffer:
            idx = self.rx_buffer.index(b'\x00')
            if idx == 0:
                # Empty frame, skip
                self.rx_buffer = self.rx_buffer[1:]
                continue

            encoded_frame = bytes(self.rx_buffer[:idx])
            self.rx_buffer = self.rx_buffer[idx+1:]

            # COBS decode
            decoded = cobs_decode(encoded_frame)
            if decoded is None or len(decoded) < 4:
                self.rx_errors += 1
                if self.verbose:
                    hex_preview = encoded_frame[:32].hex() if len(encoded_frame) > 32 else encoded_frame.hex()
                    print(f"RX: COBS decode error, len={len(encoded_frame)}, data={hex_preview}")
                continue

            # Validate CRC
            data_len = len(decoded) - 4
            rx_crc = struct.unpack('<I', decoded[data_len:])[0]
            calc_crc = crc32(decoded[:data_len])

            if rx_crc != calc_crc:
                self.crc_errors += 1
                if self.verbose:
                    data_hex = decoded[:data_len][:32].hex() if data_len > 32 else decoded[:data_len].hex()
                    crc_hex = decoded[data_len:].hex()
                    cobs_hex = encoded_frame[:20].hex() if len(encoded_frame) > 20 else encoded_frame.hex()
                    print(f"RX: CRC error (rx={rx_crc:08x} calc={calc_crc:08x}) len={data_len} crc_bytes={crc_hex}")
                    print(f"    COBS[:{len(encoded_frame)}]={cobs_hex}... data={data_hex}...")
                continue

            payload = decoded[:data_len]

            # Check for keepalive
            if payload == KEEPALIVE_PAYLOAD:
                self.keepalives_rx += 1
                if self.verbose:
                    print("RX: Keepalive")
                continue

            # Valid Ethernet frame - write to TAP
            if len(payload) >= 14:  # Minimum Ethernet frame
                try:
                    os.write(self.tap_fd, payload)
                    self.rx_frames += 1
                    if self.verbose:
                        print(f"RX: {len(payload)} bytes")
                except OSError as e:
                    self.rx_errors += 1
                    if self.verbose:
                        print(f"RX: TAP write error: {e}")

    def reconnect_serial(self):
        """Close and reopen serial port"""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        while self.running:
            try:
                self.open_serial()
                self.rx_buffer.clear()
                print("Serial reconnected")
                return True
            except Exception as e:
                print(f"Reconnect failed: {e}, retrying in 1s...")
                time.sleep(1)
        return False

    def run(self):
        """Main loop"""
        self.open_tap()
        self.configure_tap()
        self.open_serial()
        self.running = True

        last_keepalive = time.time()
        last_stats = time.time()

        print("Bridge running. Press Ctrl+C to stop.")

        try:
            while self.running:
                # Check if serial is connected
                if self.ser is None or not self.ser.is_open:
                    if not self.reconnect_serial():
                        break
                    continue

                try:
                    # Use select to wait for data on either TAP or serial
                    readable, _, _ = select.select(
                        [self.tap_fd, self.ser.fileno()],
                        [], [], 0.1
                    )

                    for fd in readable:
                        if fd == self.tap_fd:
                            # Read from TAP, send to serial
                            frame = os.read(self.tap_fd, 1600)
                            if frame:
                                self.send_frame_to_serial(frame)

                        elif fd == self.ser.fileno():
                            # Read from serial, process COBS
                            self.process_serial_rx()

                    # Send keepalive every second
                    now = time.time()
                    if now - last_keepalive >= 1.0:
                        self.send_keepalive()
                        last_keepalive = now

                    # Print stats every 10 seconds
                    if now - last_stats >= 10.0:
                        print(f"Stats: TX={self.tx_frames} RX={self.rx_frames} "
                              f"RX_err={self.rx_errors} CRC_err={self.crc_errors} "
                              f"KA_tx={self.keepalives_tx} KA_rx={self.keepalives_rx}")
                        last_stats = now

                except (serial.SerialException, OSError) as e:
                    print(f"Serial error: {e}")
                    print("Attempting to reconnect...")
                    self.reconnect_serial()

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False
            if self.ser:
                self.ser.close()
            if self.tap_fd:
                os.close(self.tap_fd)


def main():
    parser = argparse.ArgumentParser(
        description='Bridge TAP interface to serial using COBS encoding (ArduPilot compatible)'
    )
    parser.add_argument('serial_port', help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=57600, help='Baud rate (default: 57600)')
    parser.add_argument('--ip', help='IP address with netmask (e.g., 192.168.13.100/24)')
    parser.add_argument('--tap', default='tap0', help='TAP interface name (default: tap0)')
    parser.add_argument('--rtscts', action='store_true', help='Enable RTS/CTS hardware flow control')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')

    args = parser.parse_args()

    if os.geteuid() != 0:
        print("Error: Must run as root (sudo) to create TAP interface")
        sys.exit(1)

    bridge = COBSTapBridge(
        args.serial_port,
        args.baud,
        tap_name=args.tap,
        ip_addr=args.ip,
        rtscts=args.rtscts,
        verbose=args.verbose
    )
    bridge.run()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
MAVLink COBS TAP Bridge - Bridge a TAP interface to MAVLink using TUNNEL messages

This allows a Linux machine to participate in an ArduPilot COBS Ethernet network
over MAVLink, useful for long-range links where direct serial COBS isn't available.

Uses the same protocol as AP_Networking_Port_COBS:
- Keepalive: "KA" + device_id[6] + rx_good[2] + CRC32
- Data:      ethernet_frame + CRC32

Usage:
    sudo ./mavlink_cobs_bridge.py tcp:localhost:5760 --ip 192.168.13.100/24

Author: ArduPilot
License: GPLv3
"""

import argparse
import array
import fcntl
import os
import select
import struct
import subprocess
import sys
import time

# For MAVLink
try:
    from pymavlink import mavutil
except ImportError:
    print("Please install pymavlink: pip install pymavlink")
    sys.exit(1)

# Try to import numpy for faster CRC (optional)
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

# Linux TAP interface constants
TUNSETIFF = 0x400454ca
IFF_TAP = 0x0002
IFF_NO_PI = 0x1000

# MAVLink TUNNEL payload types
MAV_TUNNEL_PAYLOAD_TYPE_COBS_START = 32769
MAV_TUNNEL_PAYLOAD_TYPE_COBS_CONT = 32770
TUNNEL_PAYLOAD_MAX = 128

# Protocol constants (must match AP_Networking_COBS_Protocol.h)
KA_MARKER = b'KA'
KA_DATA_LEN = 10  # 2 marker + 6 device_id + 2 rx_good
KA_TOTAL_LEN = 14  # + 4 CRC
KEEPALIVE_INTERVAL_S = 0.5
MIN_ETH_FRAME = 14  # Minimum = Ethernet header only

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
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
]


# Convert CRC table to array for faster lookup
_CRC32_ARR = array.array('I', _CRC32_TAB)

if HAS_NUMPY:
    _CRC32_NP = np.array(_CRC32_TAB, dtype=np.uint32)

    def crc32(data):
        """Calculate CRC32 using numpy for speed"""
        crc = np.uint32(0)
        data_arr = np.frombuffer(data, dtype=np.uint8)
        for byte in data_arr:
            crc = _CRC32_NP[(crc ^ byte) & 0xff] ^ (crc >> 8)
        return int(crc)
else:
    def crc32(data):
        """Calculate CRC32 matching ArduPilot's crc_crc32(0, data, len)"""
        crc = 0
        for byte in data:
            crc = _CRC32_ARR[(crc ^ byte) & 0xff] ^ (crc >> 8)
        return crc


def cobs_encode(data):
    """COBS encode data, returns encoded bytes (without delimiter)"""
    # Pre-allocate output buffer (worst case: len + len/254 + 1)
    max_len = len(data) + (len(data) // 254) + 2
    output = bytearray(max_len)
    out_idx = 1  # Start after first code byte placeholder
    code_idx = 0
    code = 1

    for byte in data:
        if byte == 0:
            output[code_idx] = code
            code_idx = out_idx
            out_idx += 1
            code = 1
        else:
            output[out_idx] = byte
            out_idx += 1
            code += 1
            if code == 0xFF:
                output[code_idx] = code
                code_idx = out_idx
                out_idx += 1
                code = 1

    output[code_idx] = code
    return bytes(output[:out_idx])


class COBSDecoder:
    """Streaming COBS decoder matching ArduPilot's AP_Networking_COBS::Decoder"""

    __slots__ = ('state', 'code_byte', 'code_remaining', 'frame')

    def __init__(self):
        self.reset()

    def reset(self):
        """Reset decoder state for new frame"""
        self.state = 0  # 0=IDLE, 1=IN_PACKET, 2=RESYNC (use ints for speed)
        self.code_byte = 0
        self.code_remaining = 0
        self.frame = bytearray()

    def process_bytes(self, data):
        """
        Process multiple bytes at once.
        Returns list of completed frames.
        """
        frames = []
        for byte in data:
            if self.process_byte(byte):
                frames.append(bytes(self.frame))
                self.frame = bytearray()
        return frames

    def process_byte(self, byte):
        """
        Process one byte.
        Returns True when frame is complete, then call get_frame().
        """
        state = self.state

        if state == 2:  # RESYNC
            if byte == 0:
                self.state = 0  # IDLE
                self.frame = bytearray()
            return False

        if state == 0:  # IDLE
            if byte == 0:
                return False
            self.code_byte = byte
            self.code_remaining = byte - 1
            self.state = 1  # IN_PACKET
            self.frame = bytearray()
            return False

        # IN_PACKET state (state == 1)
        if byte == 0:
            if self.code_remaining == 0:
                self.state = 0  # IDLE
                return True
            else:
                self.state = 0  # IDLE
                self.frame = bytearray()
                return False

        if self.code_remaining > 0:
            self.frame.append(byte)
            self.code_remaining -= 1
        else:
            if self.code_byte < 0xFF:
                self.frame.append(0)
            self.code_byte = byte
            self.code_remaining = byte - 1

        return False

    def get_frame(self):
        """Get the decoded frame"""
        return bytes(self.frame)


def build_keepalive(device_id, rx_good):
    """Build keepalive frame: "KA" + device_id[6] + rx_good[2] + CRC32"""
    data = KA_MARKER + device_id + struct.pack('<H', rx_good & 0xFFFF)
    crc = crc32(data)
    return data + struct.pack('<I', crc)


def build_data_frame(frame):
    """Build data frame: ethernet_frame + CRC32"""
    crc = crc32(frame)
    return frame + struct.pack('<I', crc)


def identify_frame(data):
    """
    Identify frame type from decoded COBS data.
    Returns (frame_type, payload) where frame_type is 'keepalive', 'data', or None
    """
    if len(data) < 4:
        return None, None

    data_len = len(data) - 4
    rx_crc = struct.unpack('<I', data[data_len:])[0]
    payload = data[:data_len]

    # Check for keepalive
    if data_len == KA_DATA_LEN and payload[:2] == KA_MARKER:
        calc_crc = crc32(payload)
        if rx_crc == calc_crc:
            return 'keepalive', payload
        return None, None

    # Check for data frame
    if data_len >= MIN_ETH_FRAME:
        calc_crc = crc32(payload)
        if rx_crc == calc_crc:
            return 'data', payload

        # Check for ganged frame (with "GANG" suffix in CRC)
        gang_crc = crc32(payload + b'GANG')
        if rx_crc == gang_crc and data_len >= 2 + MIN_ETH_FRAME:
            # Ganged: skip seq[2]
            return 'data', payload[2:]

    return None, None


def parse_keepalive(data):
    """Extract device_id and rx_good from keepalive payload"""
    device_id = data[2:8]
    rx_good = struct.unpack('<H', data[8:10])[0]
    return device_id, rx_good


class MAVLinkCOBSBridge:
    def __init__(self, connection_string, tap_name='tap0', ip_addr=None,
                 source_system=255, source_component=0,
                 target_system=0, target_component=0,
                 verbose=False):
        self.connection_string = connection_string
        self.tap_name = tap_name
        self.ip_addr = ip_addr
        self.source_system = source_system
        self.source_component = source_component
        self.target_system = target_system
        self.target_component = target_component
        self.verbose = verbose
        self.running = False
        self.tap_fd = None
        self.mav = None

        # Local device ID (derived from TAP MAC or random)
        self.local_device_id = os.urandom(6)

        # Remote device ID (learned from keepalives)
        self.remote_device_id = None

        # RX COBS decoder
        self.decoder = COBSDecoder()

        # RX counter for keepalive stats
        self.rx_good = 0

        # Keepalive timing
        self.last_keepalive_tx = 0
        self.last_tx_time = 0

        # Stats
        self.rx_frames = 0
        self.tx_frames = 0
        self.rx_errors = 0
        self.ka_rx = 0
        self.ka_tx = 0
        self.tunnel_rx = 0
        self.tunnel_tx = 0

    def open_tap(self):
        """Create and open TAP interface"""
        tap_fd = os.open('/dev/net/tun', os.O_RDWR)

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

    def open_mavlink(self):
        """Open MAVLink connection"""
        self.mav = mavutil.mavlink_connection(
            self.connection_string,
            source_system=self.source_system,
            source_component=self.source_component
        )
        print(f"Opened MAVLink connection: {self.connection_string}")

        print("Waiting for heartbeat...")
        self.mav.wait_heartbeat()
        print(f"Got heartbeat from system {self.mav.target_system} component {self.mav.target_component}")

        if self.target_system == 0:
            self.target_system = self.mav.target_system
        if self.target_component == 0:
            self.target_component = self.mav.target_component

        self.last_keepalive_tx = time.time()
        self.last_tx_time = time.time()

        return self.mav

    def send_tunnel(self, payload_type, data):
        """Send a TUNNEL message"""
        payload = bytes(data) + bytes(128 - len(data))

        self.mav.mav.tunnel_send(
            self.target_system,
            self.target_component,
            payload_type,
            len(data),
            payload
        )
        self.tunnel_tx += 1
        self.last_tx_time = time.time()

    def send_cobs_frame(self, data):
        """COBS encode and send as fragmented TUNNEL messages"""
        encoded = cobs_encode(data) + b'\x00'

        offset = 0
        first = True

        while offset < len(encoded):
            chunk_len = min(len(encoded) - offset, TUNNEL_PAYLOAD_MAX)
            chunk = encoded[offset:offset + chunk_len]

            payload_type = MAV_TUNNEL_PAYLOAD_TYPE_COBS_START if first else MAV_TUNNEL_PAYLOAD_TYPE_COBS_CONT
            self.send_tunnel(payload_type, chunk)

            offset += chunk_len
            first = False

    def send_keepalive(self):
        """Send a protocol-compliant keepalive"""
        ka_frame = build_keepalive(self.local_device_id, self.rx_good)
        self.send_cobs_frame(ka_frame)
        self.ka_tx += 1

        if self.verbose:
            print(f"TX KA: rx_good={self.rx_good}")

    def send_frame_to_mavlink(self, frame):
        """Send Ethernet frame via MAVLink TUNNEL messages"""
        data_frame = build_data_frame(frame)
        self.send_cobs_frame(data_frame)
        self.tx_frames += 1

        if self.verbose:
            print(f"TX: {len(frame)} bytes")

    def process_tunnel_message(self, msg):
        """Process incoming TUNNEL message"""
        payload_type = msg.payload_type
        if payload_type not in (MAV_TUNNEL_PAYLOAD_TYPE_COBS_START, MAV_TUNNEL_PAYLOAD_TYPE_COBS_CONT):
            return

        self.tunnel_rx += 1

        if payload_type == MAV_TUNNEL_PAYLOAD_TYPE_COBS_START:
            self.decoder.reset()

        # Process all bytes and get any completed frames
        payload = bytes(msg.payload[:msg.payload_length])
        completed_frames = self.decoder.process_bytes(payload)

        for decoded in completed_frames:
            frame_type, frame_data = identify_frame(decoded)

            if frame_type == 'keepalive':
                device_id, rx_good = parse_keepalive(frame_data)
                if self.remote_device_id is None:
                    self.remote_device_id = device_id
                    print(f"Learned remote device ID: {device_id.hex()}")
                self.ka_rx += 1
                if self.verbose:
                    print(f"RX KA: device={device_id.hex()} rx_good={rx_good}")

            elif frame_type == 'data':
                self.rx_good += 1
                if len(frame_data) >= 14:
                    try:
                        os.write(self.tap_fd, frame_data)
                        self.rx_frames += 1
                        if self.verbose:
                            print(f"RX: {len(frame_data)} bytes")
                    except OSError as e:
                        self.rx_errors += 1
                        if self.verbose:
                            print(f"RX: TAP write error: {e}")
                else:
                    self.rx_errors += 1

            else:
                self.rx_errors += 1
                if self.verbose:
                    print("RX: Invalid frame (CRC error)")

    def reconnect_mavlink(self):
        """Close and reopen MAVLink connection"""
        if self.mav:
            try:
                self.mav.close()
            except Exception:
                pass
            self.mav = None

        while self.running:
            try:
                self.open_mavlink()
                self.decoder.reset()
                print("MAVLink reconnected")
                return True
            except Exception as e:
                print(f"Reconnect failed: {e}, retrying in 1s...")
                time.sleep(1)
        return False

    def run(self):
        """Main loop"""
        self.open_tap()
        self.configure_tap()
        self.open_mavlink()
        self.running = True

        last_stats = time.time()

        print(f"Bridge running. Local device ID: {self.local_device_id.hex()}")
        print("Press Ctrl+C to stop.")

        try:
            while self.running:
                if self.mav is None:
                    if not self.reconnect_mavlink():
                        break
                    continue

                try:
                    # Check for data on TAP (short timeout to stay responsive)
                    readable, _, _ = select.select([self.tap_fd], [], [], 0.0001)

                    for fd in readable:
                        if fd == self.tap_fd:
                            frame = os.read(self.tap_fd, 1600)
                            if frame:
                                self.send_frame_to_mavlink(frame)

                    # Drain ALL available MAVLink messages (not just one!)
                    while True:
                        msg = self.mav.recv_match(blocking=False)
                        if msg is None:
                            break
                        if msg.get_type() == 'TUNNEL':
                            self.process_tunnel_message(msg)

                    # Send keepalive periodically
                    now = time.time()
                    if now - self.last_keepalive_tx >= KEEPALIVE_INTERVAL_S:
                        self.send_keepalive()
                        self.last_keepalive_tx = now

                    # Print stats every 10 seconds
                    if now - last_stats >= 10.0:
                        print(f"Stats: TX={self.tx_frames} RX={self.rx_frames} "
                              f"ERR={self.rx_errors} KA_tx={self.ka_tx} KA_rx={self.ka_rx}")
                        last_stats = now

                except Exception as e:
                    print(f"Error: {e}")
                    print("Attempting to reconnect...")
                    self.reconnect_mavlink()

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.running = False
            if self.mav:
                self.mav.close()
            if self.tap_fd:
                os.close(self.tap_fd)


def main():
    parser = argparse.ArgumentParser(
        description='Bridge TAP interface to MAVLink using TUNNEL messages (ArduPilot COBS compatible)'
    )
    parser.add_argument('connection', help='MAVLink connection string (e.g., tcp:localhost:5760)')
    parser.add_argument('--ip', help='IP address with netmask (e.g., 192.168.13.100/24)')
    parser.add_argument('--tap', default='tap0', help='TAP interface name (default: tap0)')
    parser.add_argument('--source-system', type=int, default=255, help='MAVLink source system ID')
    parser.add_argument('--source-component', type=int, default=0, help='MAVLink source component ID')
    parser.add_argument('--target-system', type=int, default=0, help='MAVLink target system ID (0=auto)')
    parser.add_argument('--target-component', type=int, default=0, help='MAVLink target component ID (0=auto)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')

    args = parser.parse_args()

    if os.geteuid() != 0:
        print("Error: Must run as root (sudo) to create TAP interface")
        sys.exit(1)

    bridge = MAVLinkCOBSBridge(
        args.connection,
        tap_name=args.tap,
        ip_addr=args.ip,
        source_system=args.source_system,
        source_component=args.source_component,
        target_system=args.target_system,
        target_component=args.target_component,
        verbose=args.verbose
    )
    bridge.run()


if __name__ == '__main__':
    main()

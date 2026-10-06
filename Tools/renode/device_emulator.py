#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''Host-side emulated devices attached to Renode external buses.'''

import argparse
import math
import signal
import socket
import struct
import sys
import time

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PYDRONECAN = ROOT / 'modules' / 'DroneCAN' / 'pydronecan'
sys.path.insert(0, str(PYDRONECAN))

# physics air data multicast by Renode's AP_CANMcast bridge beside each bus
AIR_DATA_PORT = 57733
AIR_DATA = struct.Struct('<4sBxxxQfff')
SPECIFIC_GAS_CONSTANT = 287.05


def parse_air_data(packet):
    '''Return (airspeed m/s, pressure Pa, temperature K), or None.'''
    if len(packet) != AIR_DATA.size:
        return None
    magic, version, _, airspeed, pressure, temperature = AIR_DATA.unpack(packet)
    if (magic != b'APAD' or version != 1 or
            not all(map(math.isfinite, (airspeed, pressure, temperature))) or
            pressure <= 0 or temperature <= 0):
        return None
    return airspeed, pressure, temperature


def differential_pressure(airspeed, pressure, temperature):
    density = pressure / (SPECIFIC_GAS_CONSTANT * temperature)
    return 0.5 * density * airspeed * airspeed


class AirData:
    '''Latest physics airspeed, static pressure and temperature for a bus.'''

    def __init__(self, can_bus):
        self.airspeed = 0.0
        self.pressure = 101325.0
        self.temperature = 288.15
        group = '239.65.82.%u' % can_bus
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # as in pydronecan's mcast driver
        self.sock.bind(('0.0.0.0' if sys.platform == 'win32' else group,
                        AIR_DATA_PORT))
        try:
            self.sock.setsockopt(
                socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                struct.pack('4s4s', socket.inet_aton(group),
                            socket.inet_aton('0.0.0.0')))
        except OSError as error:
            # no multicast route: report still air
            print('no physics air data on %s: %s' % (group, error),
                  file=sys.stderr)
        self.sock.setblocking(False)

    def update(self):
        while True:
            try:
                packet = self.sock.recv(64)
            except BlockingIOError:
                return
            sample = parse_air_data(packet)
            if sample is not None:
                self.airspeed, self.pressure, self.temperature = sample

    def differential_pressure(self):
        return differential_pressure(
            self.airspeed, self.pressure, self.temperature)

    def close(self):
        self.sock.close()


def run_dronecan_airspeed(can_bus, node_id, run_seconds=None):
    import dronecan

    from dronecan import uavcan

    node = dronecan.make_node(
        'mcast:%u' % can_bus, node_id=node_id, bitrate=1000000)
    node.mode = uavcan.protocol.NodeStatus().MODE_OPERATIONAL
    node.node_info.name = b'org.ardupilot.renode.airspeed'
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    air_data = AirData(can_bus)

    def publish():
        air_data.update()
        message = uavcan.equipment.air_data.RawAirData()
        message.flags = 0
        message.static_pressure = air_data.pressure
        message.differential_pressure = air_data.differential_pressure()
        message.static_pressure_sensor_temperature = air_data.temperature
        message.differential_pressure_sensor_temperature = air_data.temperature
        message.static_air_temperature = air_data.temperature
        message.pitot_temperature = math.nan
        message.covariance = []
        node.broadcast(message)

    node.periodic(0.05, publish)
    print('DroneCAN airspeed node %u on mcast:%u' % (node_id, can_bus),
          flush=True)
    deadline = (time.monotonic() + run_seconds
                if run_seconds is not None else math.inf)
    try:
        while running and time.monotonic() < deadline:
            # The multicast driver does not block for receive timeouts. A
            # positive spin timeout therefore busy-loops inside pydronecan.
            # Poll once and sleep explicitly to keep an idle node inexpensive.
            node.spin(timeout=0)
            time.sleep(0.05)
    finally:
        air_data.close()
        node.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('device', choices=('dronecan-airspeed',))
    parser.add_argument('--can-bus', type=int, required=True)
    parser.add_argument('--node-id', type=int, required=True)
    parser.add_argument('--run-seconds', type=float,
                        help='stop after this many seconds (for tests)')
    args = parser.parse_args()
    if not 0 <= args.can_bus <= 9:
        parser.error('--can-bus must be between 0 and 9')
    if not 1 <= args.node_id <= 127:
        parser.error('--node-id must be between 1 and 127')
    if args.run_seconds is not None and args.run_seconds <= 0:
        parser.error('--run-seconds must be positive')

    if args.device == 'dronecan-airspeed':
        run_dronecan_airspeed(args.can_bus, args.node_id, args.run_seconds)
    return 0


if __name__ == '__main__':
    sys.exit(main())

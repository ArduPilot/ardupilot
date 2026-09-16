#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''Host-side emulated devices attached to Renode external buses.'''

import argparse
import math
import signal
import sys
import time

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PYDRONECAN = ROOT / 'modules' / 'DroneCAN' / 'pydronecan'
sys.path.insert(0, str(PYDRONECAN))


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

    def publish():
        message = uavcan.equipment.air_data.RawAirData()
        message.flags = 0
        message.static_pressure = 101325.0
        message.differential_pressure = 0.0
        message.static_pressure_sensor_temperature = 288.15
        message.differential_pressure_sensor_temperature = 288.15
        message.static_air_temperature = 288.15
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

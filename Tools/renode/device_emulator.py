#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''Host-side emulated devices attached to Renode external buses.'''

import argparse
import math
import signal
import sys

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PYDRONECAN = ROOT / 'modules' / 'DroneCAN' / 'pydronecan'
sys.path.insert(0, str(PYDRONECAN))


def run_dronecan_airspeed(can_bus, node_id):
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
    try:
        while running:
            node.spin(timeout=0.5)
    finally:
        node.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('device', choices=('dronecan-airspeed',))
    parser.add_argument('--can-bus', type=int, required=True)
    parser.add_argument('--node-id', type=int, required=True)
    args = parser.parse_args()
    if not 0 <= args.can_bus <= 9:
        parser.error('--can-bus must be between 0 and 9')
    if not 1 <= args.node_id <= 127:
        parser.error('--node-id must be between 1 and 127')

    if args.device == 'dronecan-airspeed':
        run_dronecan_airspeed(args.can_bus, args.node_id)
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
'''
decode RTCMStream or MovingBaselineData from a DroneCAN URI
'''

import dronecan, time
from dronecan import uavcan
from MAVProxy.modules.lib import rtcm3

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='Fix2 gap example')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--moving-baseline", default=False, action='store_true', help="decode MovingBaselineData instead of RTCMStream")
parser.add_argument("--stats", default=False, action='store_true', help="show statistics of each msg type")
parser.add_argument("port", default=None, type=str, help="serial port")
args = parser.parse_args()

# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

# Initializing a node monitor
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

rtcm_file = open("rtcm.dat", "wb")
decoder = rtcm3.RTCM3()
last_time = {}
deltas = {}
counts = {}
last_stats = 0
lost_packets = 0

def handle_RTCM(msg):
    global lost_packets
    global last_stats
    global last_time
    global deltas, counts
    data = msg.message.data.to_bytes()
    for b in data:
        if decoder.read(chr(b)):
            pkt_id = decoder.get_packet_ID()
            pkt_len = len(decoder.get_packet())
            key = (pkt_id,pkt_len)
            if not key in counts:
                counts[key] = 1
            else:
                counts[key] += 1
            now = time.time()
            if not key in last_time:
                last_time[key] = now
            else:
                dt = now - last_time[key]
                last_time[key] = now
                if not key in deltas:
                    deltas[key] = dt
                else:
                    deltas[key] = 0.9 * deltas[key] + 0.1 * dt
            if args.stats:
                if now - last_stats >= 2.0:
                    last_stats = now
                    print("\nLost packets %u" % lost_packets)
                    total_len = 0
                    for key in sorted(deltas.keys()):
                        pkt_id, pkt_len = key
                        if not key in counts:
                            counts[key] = 0
                            rate = 0.0
                        else:
                            rate = 1.0/deltas[key]
                        print("%u len=%u %.1fHz count=%u" % (pkt_id, pkt_len, rate, counts[key]))
                        if counts[key] > 0:
                            total_len += pkt_len
                    print("Total len %u" % total_len)
                    for key in sorted(deltas.keys()):
                        pkt_id, pkt_len = key
                        if counts[key] == 0:
                            deltas.pop(key)
                            last_time.pop(key)
                    counts = {}
            else:
                print("packet len %u ID %u" % (pkt_len, pkt_id))
        elif len(decoder.pkt) == 0:
            lost_packets += 1
    rtcm_file.write(data)
    rtcm_file.flush()

# callback for printing ESC status message to stdout in human-readable YAML format.
if args.moving_baseline:
    node.add_handler(dronecan.ardupilot.gnss.MovingBaselineData, handle_RTCM)
else:
    node.add_handler(dronecan.uavcan.equipment.gnss.RTCMStream, handle_RTCM)

while True:
    try:
        node.spin()
    except Exception as ex:
        pass

#!/usr/bin/env python3
import dronecan
import sys
# progress bar
from tqdm import tqdm

if len(sys.argv) != 4:
    print("Usage: python3 canlog2gps.py <messageName> <logfile> <outputfile>")
    sys.exit(1)

# Create a CAN bus instance
global node
print('Starting server as Node ID', 10)

filename = sys.argv[2]
out_file = open(sys.argv[3],'wb')
node = dronecan.make_node("filein:"+filename, node_id=10, bitrate=1000000,speedup=-1)

def handle_Data(msg):
    # dump the message data to a file
    out_file.write(msg.message.data.to_bytes())

# message type by key name
message_type = getattr(dronecan.ardupilot.gnss, sys.argv[1])
node.add_handler(message_type, handle_Data)

# progress bar
pbar = tqdm(total=100)

while True:
    try:
        node.spin(0.001)
        # print(node.can_driver.stream_progress())
        # show the progress of the stream
        pbar.update(node.can_driver.stream_progress() - pbar.n)
        if node.can_driver.end_of_stream():
            print('End of stream')
            out_file.flush()
            out_file.close()
            sys.exit(0)
    except KeyboardInterrupt:
        out_file.flush()
        out_file.close()
        sys.exit(0)

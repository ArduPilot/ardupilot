#!/usr/bin/env python3
from utils import get_serial_master
import time
import sys
import os

if __name__ == '__main__':
    port = "/dev/ttyTHS1"
    timeout = 30
    if len(sys.argv) > 1:
        port = sys.argv[1]

    if len(sys.argv) > 2:
        timeout = int(sys.argv[2], 10)

    start_time = time.time()

    while True:

        if (time.time() - start_time) > timeout:
            print(f"Timeout {timeout} expired")
            break

        print(f"Waiting for ardupilot to become online at {port}")
        try:
            some_master = get_serial_master(port)
            if some_master is not None:
                print("OK")
                sys.exit(os.EX_OK)
        except Exception as e:
            print(f"Something went wrong: {repr(e)}")
            time.sleep(1)

    print("Cannot connect to autopilot")
    sys.exit(1)

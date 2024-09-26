#!/usr/bin/env python3
from utils import get_serial_master, get_tcp_master
import time
import sys, os
import serial

if __name__ == '__main__':
    port = "/dev/ttyTHS1"
    if len(sys.argv) > 1:
        port = sys.argv[1]

    while True:
        print(f"Waiting for ardupilot to become online at {port}")
        try:
            some_master = get_serial_master(port)
            if some_master is not None:
                break
        except Exception as e:
            print(f"Something went wrong: {repr(e)}")
            pass
    
        time.sleep(1)

    print("OK")

#!/usr/bin/env python3
from utils import get_serial_master, get_tcp_master
import time
import sys, os

if __name__ == '__main__':
    while True:
        print(f"Waiting for ardupilot to become online at {sys.argv[1]}")
        some_master = get_serial_master(sys.argv[1])
        if some_master is not None:
            break
        time.sleep(1)

    print("OK")

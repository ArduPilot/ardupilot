#!/usr/bin/env python3
from utils import get_serial_master, get_tcp_master
import time
import sys, os
import serial

if __name__ == '__main__':
    while True:
        print(f"Waiting for ardupilot to become online at {sys.argv[1]}")
        try:
            some_master = get_serial_master(sys.argv[1])
            if some_master is not None:
                break
        except Exception as e:
            print(f"Something went wrong: {repr(e)}")
            pass
    
        time.sleep(1)

    print("OK")

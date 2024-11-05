#!/usr/bin/env python3
from utils import get_serial_master, get_tcp_master
import sys, os

if __name__ == '__main__':
    port = "/dev/ttyTHS1"
    if len(sys.argv) > 1:
        port = sys.argv[1]

    some_master = get_serial_master(port)

    assert some_master is not None
    print("OK")
    
    some_master.reboot_autopilot()
    
    print("Rebooting ..")

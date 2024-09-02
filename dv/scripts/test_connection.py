#!/usr/bin/env python3
from utils import get_serial_master, get_tcp_master

if __name__ == '__main__':
    some_master = get_serial_master("/dev/ttyTHS1")
    assert some_master is not None
    print("OK")

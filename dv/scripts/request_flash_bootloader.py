#!/usr/bin/env python3

import sys

from utils import get_serial_master

import utils

if __name__ == "__main__":

    port = "/dev/ttyTHS1"
    if len(sys.argv) > 1:
        port = sys.argv[1]

    some_master = get_serial_master(port)
    assert some_master is not None
    print("OK")

    res = utils.flash_bootloader(some_master)

    sys.exit(not res)

#!/usr/bin/env python3
from utils import get_serial_master, get_tcp_master
import param_manip as pm

if __name__ == '__main__':
    some_master = get_serial_master("/dev/ttyTHS1")
    assert some_master is not None
    print("OK")

    pm.write_param(some_master, "FORMAT_VERSION", 0)

    some_master.reboot_autopilot()

    print("Rebooting ..")


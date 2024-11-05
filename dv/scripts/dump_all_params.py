#!/usr/bin/env python3
from utils import get_serial_master, get_tcp_master
import param_manip as pm

if __name__ == '__main__':
    some_master = get_serial_master("/dev/ttyTHS1")
    assert some_master is not None
    all_params = pm.get_all_params_from_device(some_master)
    print(all_params)
    pm.write_params_to_file(all_params, "params.csv")

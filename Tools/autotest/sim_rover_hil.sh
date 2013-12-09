#!/bin/bash

set -x

pkill -f sim_rover.py
set -e

gnome-terminal -e "nice ../Tools/autotest/pysim/sim_rover.py --home=40.071374969556928,-105.22978898137808,1583.702759,246 --rate=400"
sleep 2
mavproxy.py --aircraft=test --out 127.0.0.1:14550 --load-module=HIL $*

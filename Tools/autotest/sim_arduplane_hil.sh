#!/bin/bash

killall -q JSBSim
pkill -f runsim.py

set -e
set -x

gnome-terminal -e '../Tools/autotest/jsbsim/runsim.py --home=-35.362938,149.165085,584,270'
sleep 2

mavproxy.py --out 127.0.0.1:14550 --load-module=HIL $*

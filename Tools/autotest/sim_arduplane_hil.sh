#!/bin/bash

killall -q JSBSim
pkill -f runsim.py

set -e
set -x

autotest=$(dirname $(readlink -e $0))
cmd="../Tools/autotest/jsbsim/runsim.py --home=-35.362938,149.165085,584,270"
$autotest/run_in_terminal_window.sh "simulator" $cmd || exit 1

sleep 2

mavproxy.py --out 127.0.0.1:14550 --load-module=HIL $*


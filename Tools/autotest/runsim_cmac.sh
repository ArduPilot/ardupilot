#!/bin/bash

# useful example script for HIL testing with ArduPlane

set -x

killall -q JSBSim
pkill -f runsim.py
set -e

autotest=$(dirname $(readlink -e $0))
pushd $autotest/../../ArduPlane
../Tools/autotest/jsbsim/runsim.py --home=-35.362942,149.165193,585,354

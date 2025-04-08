#!/usr/bin/env bash

# assume we start the script from the root directory
ROOTDIR=$PWD
$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-tri --add-param-file=libraries/SITL/examples/Webots/tricopter.parm 

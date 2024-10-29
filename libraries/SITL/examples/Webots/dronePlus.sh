#!/usr/bin/env bash

# assume we start the script from the root directory
ROOTDIR=$PWD
$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-quad --add-param-file=libraries/SITL/examples/Webots/quadPlus.parm 

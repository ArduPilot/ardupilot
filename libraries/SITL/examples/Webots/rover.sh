#!/bin/bash

# assume we start the script from the root directory
ROOTDIR=$PWD
$PWD/Tools/autotest/sim_vehicle.py -v APMrover2 -w --model webots-rover --add-param-file=libraries/SITL/examples/Webots/rover.parm 

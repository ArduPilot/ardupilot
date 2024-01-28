#!/bin/bash

# assume we start the script from the root directory
ROOTDIR=$PWD
$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-tri:127.0.0.1:5577  --add-param-file=$PWD/libraries/SITL/examples/Webots/webots_2023a/tricopter_2023a.parm --out=udpout:127.0.0.1:14550 -N  

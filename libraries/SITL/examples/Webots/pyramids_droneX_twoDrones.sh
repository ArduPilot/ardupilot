#!/usr/bin/env bash

# assume we start the script from the root directory
ROOTDIR=$PWD
xterm -title "Quad 1"  -e "$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --instance 10 --out=udpout:127.0.0.1:14450 --model webots-quad:127.0.0.1:5599 --add-param-file=libraries/SITL/examples/Webots/quadX.parm  -L Pyramid "  &
xterm -title "Quad 2"  -e "$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --instance 20 --out=udpout:127.0.0.1:14550 --model webots-quad:127.0.0.1:5598 --add-param-file=libraries/SITL/examples/Webots/quadX2.parm -L Pyramid "  &

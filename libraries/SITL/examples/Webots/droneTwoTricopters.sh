#!/usr/bin/env bash

# assume we start the script from the root directory



ROOTDIR=$PWD


xterm -title "TriCopter 1"  -e "$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --instance 10 --out=udpout:127.0.0.1:14450 --model webots-tri:127.0.0.1:5599 --add-param-file=libraries/SITL/examples/Webots/tricopter.parm "  &
xterm -title "TriCopter 2"  -e "$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --instance 20 --out=udpout:127.0.0.1:14550 --model webots-tri:127.0.0.1:5598 --add-param-file=libraries/SITL/examples/Webots/tricopter2.parm "  &

#!/usr/bin/env bash

# run from the repository root
$PWD/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-quad:127.0.0.1:5599 --add-param-file=libraries/SITL/examples/Webots/quadX.parm -L Pyramid

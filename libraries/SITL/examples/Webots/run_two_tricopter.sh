#!/usr/bin/env bash

# run from the repository root
ROOTDIR=$PWD
WEBOTS=$ROOTDIR/libraries/SITL/examples/Webots

source "$WEBOTS/multi_vehicle_launch.sh"

# -N: do not rebuild; run "./waf copter" once manually instead. Both instances
# would otherwise race waf in the same build dir.
launch_sitl "TriCopter 1" webots-tri 5599 -v ArduCopter -w -N --instance 10 --out=udpout:127.0.0.1:14450 --add-param-file="$WEBOTS/tricopter.parm"
launch_sitl "TriCopter 2" webots-tri 5598 -v ArduCopter -w -N --instance 20 --out=udpout:127.0.0.1:14550 --add-param-file="$WEBOTS/tricopter2.parm"

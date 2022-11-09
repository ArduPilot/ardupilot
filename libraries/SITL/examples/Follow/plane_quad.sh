#!/bin/bash

# a quad following a plane

# assume we start the script from the root directory
ROOTDIR=$PWD
COPTER=$ROOTDIR/build/sitl/bin/arducopter
PLANE=$ROOTDIR/build/sitl/bin/arduplane

[ -x "$COPTER" -a -x "$PLANE" ] || {
    ./waf configure --board sitl
    ./waf plane copter
}

# setup for either TCP or multicast
#UARTA="tcp:0"
UARTA="mcast:"

PLANE_DEFAULTS="$ROOTDIR/Tools/autotest/models/plane.parm"
COPTER_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/copter.parm"

mkdir -p swarm/plane swarm/copter
(cd swarm/plane && $PLANE --model plane --uartA $UARTA --defaults $PLANE_DEFAULTS) &

# create default parameter file for the follower
cat <<EOF > swarm/copter/follow.parm
SYSID_THISMAV 2
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_TYPE 1
FOLL_SYSID 1
FOLL_DIST_MAX 1000
EOF

(cd swarm/copter && $COPTER --model quad --uartA $UARTA --instance 1 --defaults $COPTER_DEFAULTS,follow.parm) &
wait

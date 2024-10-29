#!/usr/bin/env bash

# assume we start the script from the root directory
ROOTDIR=$PWD
ROVER=$ROOTDIR/build/sitl/bin/ardurover

GCS_IP=192.168.2.48

BASE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/rover.parm,$ROOTDIR/Tools/autotest/default_params/rover-skid.parm"

# start up main rover in the current directory
$ROVER --model morse-skid --serial0 udpclient:$GCS_IP --serial1 mcast: --defaults $BASE_DEFAULTS &

# now start 2 rovers to follow the first, using
# a separate directory for each to keep the eeprom.bin
# and logs separate
for i in $(seq 2); do
    echo "Starting rover $i"
    port1=$(expr 60000 + $i \* 2)
    port2=$(expr 60001 + $i \* 2)
    mkdir -p rov$i

    SYSID=$(expr $i + 1)
    FOLL_SYSID=$(expr $SYSID - 1)

    # create default parameter file for the follower
    cat <<EOF > rov$i/follow.parm
SYSID_THISMAV $SYSID
SERVO1_FUNCTION 73
SERVO3_FUNCTION 74
INITIAL_MODE 6
MODE6 6
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_TYPE 1
FOLL_SYSID $FOLL_SYSID
FOLL_DIST_MAX 1000
EOF
    pushd rov$i
    $ROVER --model "morse-skid:127.0.0.1:$port1:$port2" --serial0 tcp:0 --serial1 mcast: --instance $i --defaults $BASE_DEFAULTS,follow.parm &
    popd
done
wait

#!/bin/bash

# assume we start the script from the root directory
ROOTDIR=$PWD
COPTER=$ROOTDIR/build/sitl/bin/arducopter

GCS_IP=$1

BASE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/copter.parm,$ROOTDIR/libraries/SITL/examples/Airsim/quadX.parm"

[ -x "$COPTER" ] || {
	./waf configure --board sitl
	./waf copter
}

# start up main rover in the current directory
$COPTER --model airsim-copter --uartA udpclient:$GCS_IP --uartC mcast: --defaults $BASE_DEFAULTS &

# now start another copter to follow the first, using
# a separate directory to keep the eeprom.bin and logs separate
# for increasing the number of copters, change the number in seq
for i in $(seq 1); do
    echo "Starting copter $i"
    mkdir -p copter$i

    SYSID=$(expr $i + 1)
    FOLL_SYSID=$(expr $SYSID - 1)

    # create default parameter file for the follower
    cat <<EOF > copter$i/follow.parm
SYSID_THISMAV $SYSID
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_TYPE 1
FOLL_SYSID $FOLL_SYSID
FOLL_DIST_MAX 1000
EOF
    pushd copter$i
    $COPTER --model airsim-copter --uartA tcp:0 --uartC mcast: --instance $i --defaults $BASE_DEFAULTS,follow.parm &
    popd
done
wait

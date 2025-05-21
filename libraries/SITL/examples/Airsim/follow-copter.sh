#!/usr/bin/env bash

# Example script for multi-vehicle simulation with AirSim
# see https://ardupilot.org/dev/docs/sitl-with-airsim.html#multi-vehicle-simulation for details

# Usage - From ardupilot root directory, run - libraries/SITL/examples/Airsim/follow-copter.sh $GCS_IP
# $GCS_IP is the IP address of the system running the GCs, by default is 127.0.0.1

# Kill all SITL binaries when exiting
trap "killall -9 arducopter" SIGINT SIGTERM EXIT

# assume we start the script from the root directory
ROOTDIR=$PWD
COPTER=$ROOTDIR/build/sitl/bin/arducopter

# Set GCS_IP address
if [ -z $1 ]; then
    GCS_IP="127.0.0.1"
else
    GCS_IP=$1
fi

# Check if Platform is Native Linux, WSL or Cygwin
# Needed for correct multicast addressing
unameOut="$(uname -s)"

if [ "$(expr substr $unameOut 1 5)" == "Linux" ]; then
    # Check for WSL
    if grep -q Microsoft /proc/version; then
        MCAST_IP_PORT="127.0.0.1:14550"

    # Native Linux
    else
        MCAST_IP_PORT=""                    # Use default IP, port
    fi

elif [ "$(expr substr $unameOut 1 6)" == "CYGWIN" ]; then
    MCAST_IP_PORT="0.0.0.0:14550"
fi

BASE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/copter.parm,$ROOTDIR/Tools/autotest/default_params/airsim-quadX.parm"

[ -x "$COPTER" ] || {
	./waf configure --board sitl
	./waf copter
}

# start up main copter in the current directory
$COPTER --model airsim-copter --serial0 udpclient:$GCS_IP --serial1 mcast:$MCAST_IP_PORT --defaults $BASE_DEFAULTS &

# Set number of extra copters to be simulated, change this for increasing the count
NCOPTERS="1"


# now start another copter to follow the first, using
# a separate directory to keep the eeprom.bin and logs separate
for i in $(seq $NCOPTERS); do
    echo "Starting copter $i"
    mkdir -p copter$i

    SYSID=$(expr $i + 1)
    FOLL_SYSID=$(expr $SYSID - 1)

    # create default parameter file for the follower
    cat <<EOF > copter$i/follow.parm
MAV_SYSID $SYSID
FOLL_ENABLE 1
FOLL_OFS_X -5
FOLL_OFS_TYPE 1
FOLL_SYSID $FOLL_SYSID
FOLL_DIST_MAX 1000
EOF
    pushd copter$i
    $COPTER --model airsim-copter --serial0 tcp:0 --serial1 mcast:$MCAST_IP_PORT --instance $i --defaults $BASE_DEFAULTS,follow.parm &
    popd
done
wait

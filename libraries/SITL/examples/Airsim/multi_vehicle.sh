#!/usr/bin/env bash

# Example script for multi-vehicle simulation with AirSim and usage with ROS
# see https://ardupilot.org/dev/docs/sitl-with-airsim.html#multi-vehicle-simulation for details

# Usage - From ardupilot root directory, run - libraries/SITL/examples/Airsim/multi_vehicle.sh $GCS_IP
# $GCS_IP is the IP address of the system running the GCS, by default is 127.0.0.1

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

BASE_DEFAULTS="$ROOTDIR/Tools/autotest/default_params/copter.parm,$ROOTDIR/Tools/autotest/default_params/airsim-quadX.parm"

[ -x "$COPTER" ] || {
    ./waf configure --board sitl
    ./waf copter
}

# start up main copter in the current directory
$COPTER --model airsim-copter --serial0 udpclient:$GCS_IP:14550 --serial3 tcp:0 --defaults $BASE_DEFAULTS &

# Set number of "extra" copters to be simulated, change this for increasing the count
NCOPTERS="1"


# now start another copter, using
# a separate directory to keep the eeprom.bin and logs separate
for i in $(seq $NCOPTERS); do
    echo "Starting copter $i"
    mkdir -p copter$i

    SYSID=$(expr $i + 1)
    UDP_PORT=$((14550 + $i * 10))

    # create seperate parameter file for each drone for SYSID
    cat <<EOF > copter$i/identity.parm
MAV_SYSID $SYSID
EOF

    pushd copter$i
    $COPTER --model airsim-copter --serial0 udpclient:$GCS_IP:$UDP_PORT --serial3 tcp:$i \
            --instance $i --defaults $BASE_DEFAULTS,identity.parm &
    popd
done
wait

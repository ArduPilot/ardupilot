#!/bin/bash

# home location lat, lon, alt, heading
LOCATION="CMAC"
TRACKER_LOCATION="CMAC_PILOTSBOX"
VEHICLE=""
BUILD_TARGET="sitl"
FRAME=""
NUM_PROCS=1

# check the instance number to allow for multiple copies of the sim running at once
INSTANCE=0
USE_VALGRIND=0
USE_GDB=0
USE_GDB_STOPPED=0
CLEAN_BUILD=0
START_ANTENNA_TRACKER=0
WIPE_EEPROM=0
REVERSE_THROTTLE=0
NO_REBUILD=0
START_HIL=0
TRACKER_ARGS=""
EXTERNAL_SIM=0

usage()
{
cat <<EOF
Usage: sim_vehicle.sh [options] [mavproxy_options]
Options:
    -v VEHICLE       vehicle type (ArduPlane, ArduCopter or APMrover2)
                     vehicle type defaults to working directory
    -I INSTANCE      instance of simulator (default 0)
    -V               enable valgrind for memory access checking (very slow!)
    -G               use gdb for debugging ardupilot
    -g               use gdb for debugging ardupilot, but don't auto-start
    -T               start an antenna tracker instance
    -A               pass arguments to antenna tracker
    -t               set antenna tracker start location
    -L               select start location from Tools/autotest/locations.txt
    -c               do a make clean before building
    -N               don't rebuild before starting ardupilot
    -w               wipe EEPROM and reload parameters
    -R               reverse throttle in plane
    -f FRAME         set aircraft frame type
                     for copters can choose +, X, quad or octa
                     for planes can choose elevon or vtail
    -j NUM_PROC      number of processors to use during build (default 1)
    -H               start HIL
    -e               use external simulator

mavproxy_options:
    --map            start with a map
    --console        start with a status console
    --out DEST       start MAVLink output to DEST

Note: 
    eeprom.bin in the starting directory contains the parameters for your 
    simulated vehicle. Always start from the same directory. It is recommended that 
    you start in the main vehicle directory for the vehicle you are simulating, 
    for example, start in the ArduPlane directory to simulate ArduPlane
EOF
}


# parse options. Thanks to http://wiki.bash-hackers.org/howto/getopts_tutorial
while getopts ":I:VgGcj:TA:t:L:v:hwf:RNHe" opt; do
  case $opt in
    v)
      VEHICLE=$OPTARG
      ;;
    I)
      INSTANCE=$OPTARG
      ;;
    V)
      USE_VALGRIND=1
      ;;
    N)
      NO_REBUILD=1
      ;;
    H)
      START_HIL=1
      NO_REBUILD=1
      ;;
    T)
      START_ANTENNA_TRACKER=1
      ;;
    A)
      TRACKER_ARGS="$OPTARG"
      ;;
    R)
      REVERSE_THROTTLE=1
      ;;
    G)
      USE_GDB=1
      ;;
    g)
      USE_GDB=1
      USE_GDB_STOPPED=1
      ;;
    L)
      LOCATION="$OPTARG"
      ;;
    f)
      FRAME="$OPTARG"
      ;;
    t)
      TRACKER_LOCATION="$OPTARG"
      ;;
    c)
      CLEAN_BUILD=1
      ;;
    j)
      NUM_PROCS=$OPTARG
      ;;
    w)
      WIPE_EEPROM=1
      ;;
    e)
      EXTERNAL_SIM=1
      ;;
    h)
      usage
      exit 0
      ;;
    \?)
      # allow other args to pass on to mavproxy
      break
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      usage
      exit 1
  esac
done
shift $((OPTIND-1))

# kill existing copy if this is the '0' instance only
kill_tasks() 
{
    [ "$INSTANCE" -eq "0" ] && {
        killall -q JSBSim lt-JSBSim ArduPlane.elf ArduCopter.elf APMrover2.elf AntennaTracker.elf
        pkill -f runsim.py
        pkill -f sim_tracker.py
        pkill -f sim_rover.py
        pkill -f sim_multicopter.py
    }
}

kill_tasks

trap kill_tasks SIGINT

# setup ports for this instance
MAVLINK_PORT="tcp:127.0.0.1:"$((5760+10*$INSTANCE))
SIMIN_PORT="127.0.0.1:"$((5502+10*$INSTANCE))
SIMOUT_PORT="127.0.0.1:"$((5501+10*$INSTANCE))
FG_PORT="127.0.0.1:"$((5503+10*$INSTANCE))

set -x

[ -z "$VEHICLE" ] && {
    VEHICLE=$(basename $PWD)
}

EXTRA_PARM=""
EXTRA_SIM=""

# modify build target based on copter frame type
case $FRAME in
    +|quad)
	BUILD_TARGET="sitl"
        EXTRA_SIM="--frame=quad"
	;;
    X)
	BUILD_TARGET="sitl"
        EXTRA_PARM="param set FRAME 1;"
        EXTRA_SIM="--frame=X"
	;;
    octa)
	BUILD_TARGET="sitl-octa"
        EXTRA_SIM="--frame=octa"
	;;
    elevon*)
        EXTRA_PARM="param set ELEVON_OUTPUT 4;"
        EXTRA_SIM="--elevon"
	;;
    vtail)
        EXTRA_PARM="param set VTAIL_OUTPUT 4;"
        EXTRA_SIM="--vtail"
	;;
    skid)
        EXTRA_SIM="--skid-steering"
	;;
    obc)
        BUILD_TARGET="sitl-obc"
	;;
    "")
        ;;
    *)
        echo "Unknown frame type $FRAME"
        usage
        exit 1
        ;;
esac

autotest=$(dirname $(readlink -e $0))
if [ $NO_REBUILD == 0 ]; then
pushd $autotest/../../$VEHICLE || {
    echo "Failed to change to vehicle directory for $VEHICLE"
    usage
    exit 1
}
if [ $CLEAN_BUILD == 1 ]; then
    make clean
fi
make $BUILD_TARGET -j$NUM_PROCS || {
    make clean
    make $BUILD_TARGET -j$NUM_PROCS
}
popd
fi

# get the location information
SIMHOME=$(cat $autotest/locations.txt | grep -i "^$LOCATION=" | cut -d= -f2)
[ -z "$SIMHOME" ] && {
    echo "Unknown location $LOCATION"
    usage
    exit 1
}
echo "Starting up at $LOCATION : $SIMHOME"

TRACKER_HOME=$(cat $autotest/locations.txt | grep -i "^$TRACKER_LOCATION=" | cut -d= -f2)
[ -z "$TRACKER_HOME" ] && {
    echo "Unknown tracker location $TRACKER_LOCATION"
    usage
    exit 1
}


if [ $START_ANTENNA_TRACKER == 1 ]; then
    pushd $autotest/../../AntennaTracker
    if [ $CLEAN_BUILD == 1 ]; then
        make clean
    fi
    make sitl -j$NUM_PROCS || {
        make clean
        make sitl -j$NUM_PROCS
    }
    TRACKER_INSTANCE=1
    TRACKIN_PORT="127.0.0.1:"$((5502+10*$TRACKER_INSTANCE))
    TRACKOUT_PORT="127.0.0.1:"$((5501+10*$TRACKER_INSTANCE))
    TRACKER_UARTA="tcp:127.0.0.1:"$((5760+10*$TRACKER_INSTANCE))
    cmd="nice /tmp/AntennaTracker.build/AntennaTracker.elf -I1"
    $autotest/run_in_terminal_window.sh "AntennaTracker" $cmd || exit 1
    $autotest/run_in_terminal_window.sh "pysim(Tracker)" nice $autotest/pysim/sim_tracker.py --home=$TRACKER_HOME --simin=$TRACKIN_PORT --simout=$TRACKOUT_PORT $TRACKER_ARGS || exit 1
    popd
fi

cmd="/tmp/$VEHICLE.build/$VEHICLE.elf -I$INSTANCE"
if [ $WIPE_EEPROM == 1 ]; then
    cmd="$cmd -w"
fi

case $VEHICLE in
    ArduPlane)
        [ "$REVERSE_THROTTLE" == 1 ] && {
            EXTRA_SIM="$EXTRA_SIM --revthr"
        }
        RUNSIM="nice $autotest/jsbsim/runsim.py --home=$SIMHOME --simin=$SIMIN_PORT --simout=$SIMOUT_PORT --fgout=$FG_PORT $EXTRA_SIM"
        PARMS="ArduPlane.parm"
        if [ $WIPE_EEPROM == 1 ]; then
            cmd="$cmd -PFORMAT_VERSION=13 -PSKIP_GYRO_CAL=1 -PRC3_MIN=1000 -PRC3_TRIM=1000"
        fi
        ;;
    ArduCopter)
        RUNSIM="nice $autotest/pysim/sim_multicopter.py --home=$SIMHOME --simin=$SIMIN_PORT --simout=$SIMOUT_PORT --fgout=$FG_PORT $EXTRA_SIM"
        PARMS="copter_params.parm"
        ;;
    APMrover2)
        RUNSIM="nice $autotest/pysim/sim_rover.py --home=$SIMHOME --rate=400 $EXTRA_SIM"
        PARMS="Rover.parm"
        ;;
    *)
        echo "Unknown vehicle simulation type $VEHICLE - please specify vehicle using -v VEHICLE_TYPE"
        usage
        exit 1
        ;;
esac

if [ $START_HIL == 0 ]; then
if [ $USE_VALGRIND == 1 ]; then
    echo "Using valgrind"
    $autotest/run_in_terminal_window.sh "ardupilot (valgrind)" valgrind $cmd || exit 1
elif [ $USE_GDB == 1 ]; then
    echo "Using gdb"
    tfile=$(mktemp)
    [ $USE_GDB_STOPPED == 0 ] && {
        echo r > $tfile
    }
    $autotest/run_in_terminal_window.sh "ardupilot (gdb)" gdb -x $tfile --args $cmd || exit 1
else
    $autotest/run_in_terminal_window.sh "ardupilot" $cmd || exit 1
fi
fi

trap kill_tasks SIGINT

sleep 2
rm -f $tfile
if [ $EXTERNAL_SIM == 0 ]; then
    $autotest/run_in_terminal_window.sh "Simulator" $RUNSIM || {
        echo "Failed to start simulator: $RUNSIM"
        exit 1
    }
    sleep 2
else
    echo "Using external simulator"
fi

# mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 
options=""
if [ $START_HIL == 0 ]; then
options="--master $MAVLINK_PORT --sitl $SIMOUT_PORT"
fi
options="$options --out 127.0.0.1:14550 --out 127.0.0.1:14551"
extra_cmd1=""
if [ $WIPE_EEPROM == 1 ]; then
    extra_cmd="param forceload $autotest/$PARMS; $EXTRA_PARM; param fetch"
fi
if [ $START_ANTENNA_TRACKER == 1 ]; then
    options="$options --load-module=tracker"
    extra_cmd="$extra_cmd module load map; tracker set port $TRACKER_UARTA; tracker start;"
fi
if [ $START_HIL == 1 ]; then
    options="$options --load-module=HIL"
fi
mavproxy.py $options --cmd="$extra_cmd" $*
kill_tasks

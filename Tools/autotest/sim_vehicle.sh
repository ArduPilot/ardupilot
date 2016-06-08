#!/bin/bash

# home location lat, lon, alt, heading
LOCATION="CMAC"
TRACKER_LOCATION="CMAC_PILOTSBOX"
VEHICLE=""
BUILD_TARGET="sitl"
FRAME=""
NUM_PROCS=1
SPEEDUP="1"

# check the instance number to allow for multiple copies of the sim running at once
INSTANCE=0
USE_VALGRIND=0
USE_GDB=0
USE_GDB_STOPPED=0
DEBUG_BUILD=0
USE_MAVLINK_GIMBAL=0
CLEAN_BUILD=0
START_ANTENNA_TRACKER=0
WIPE_EEPROM=0
REVERSE_THROTTLE=0
NO_REBUILD=0
START_HIL=0
SITLRCIN=1
EXTRA_ARGS=""
MODEL=""
BREAKPOINT=""
OVERRIDE_BUILD_TARGET=""
DELAY_START=0
DEFAULTS_PATH=""
MAVLINK_PROTOCOL_VERSION="1"

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
    -D               build with debugging
    -B               add a breakpoint at given location in debugger
    -T               start an antenna tracker instance
    -A               pass arguments to SITL instance
    -t               set antenna tracker start location
    -L               select start location from Tools/autotest/locations.txt
    -l               set the custom start location from -L
    -c               do a make clean before building
    -N               don't rebuild before starting ardupilot
    -w               wipe EEPROM and reload parameters
    -R               reverse throttle in plane
    -M               enable MAVLink gimbal
    -f FRAME         set aircraft frame type
                     for copters can choose +, X, quad or octa
                     for planes can choose elevon or vtail
    -b BUILD_TARGET  override SITL build target
    -j NUM_PROC      number of processors to use during build (default 1)
    -H               start HIL
    -S SPEEDUP       set simulation speedup (1 for wall clock time)
    -d TIME          delays the start of mavproxy by the number of seconds
    -P VERSION       mavlink protocol version (1 or 2)

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
while getopts ":I:VgGcj:TA:t:L:l:v:hwf:RNHeMS:DB:b:d:P:" opt; do
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
      EXTRA_ARGS="$OPTARG"
      ;;
    R)
      REVERSE_THROTTLE=1
      ;;
    G)
      USE_GDB=1
      ;;
    D)
      DEBUG_BUILD=1
      ;;
    d)
      DELAY_START="$OPTARG"
      ;;
    B)
      BREAKPOINT="$OPTARG"
      ;;
    M)
      USE_MAVLINK_GIMBAL=1
      ;;
    g)
      USE_GDB=1
      USE_GDB_STOPPED=1
      ;;
    L)
      LOCATION="$OPTARG"
      ;;
    l)
      CUSTOM_LOCATION="$OPTARG"
      ;;
    f)
      FRAME="$OPTARG"
      ;;
    S)
      SPEEDUP="$OPTARG"
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
    b)
      OVERRIDE_BUILD_TARGET="$OPTARG"
      ;;
    P)
      MAVLINK_PROTOCOL_VERSION="$OPTARG"
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
	for pname in JSBSim lt-JSBSim ArduPlane.elf ArduCopter.elf APMrover2.elf AntennaTracker.elf JSBSIm.exe MAVProxy.exe; do
	    pkill "$pname"
	done
        pkill -f runsim.py
    }
}

if [ $START_HIL == 0 ]; then
kill_tasks
fi

trap kill_tasks SIGINT

# setup ports for this instance
MAVLINK_PORT="tcp:127.0.0.1:"$((5760+10*$INSTANCE))
SIMOUT_PORT="127.0.0.1:"$((5501+10*$INSTANCE))

[ -z "$VEHICLE" ] && {
    CDIR="$PWD"
    rpath=$(which realpath)
    [ -n "$rpath" ] && {
        CDIR=$(realpath $CDIR)
    }
    VEHICLE=$(basename $CDIR)
}

[ -z "$FRAME" -a "$VEHICLE" = "APMrover2" ] && {
    FRAME="rover"
}

[ -z "$FRAME" -a "$VEHICLE" = "ArduPlane" ] && {
    FRAME="jsbsim"
}
[ -z "$FRAME" -a "$VEHICLE" = "ArduCopter" ] && {
    FRAME="quad"
}
[ -z "$FRAME" -a "$VEHICLE" = "AntennaTracker" ] && {
    FRAME="tracker"
}

EXTRA_PARM=""

check_jsbsim_version()
{
    jsbsim_version=$(JSBSim --version)
    if [[ $jsbsim_version != *"ArduPilot"* ]]
    then
        cat <<EOF
=========================================================
You need the latest ArduPilot version of JSBSim installed
and in your \$PATH

Please get it from git://github.com/tridge/jsbsim.git
See 
  http://dev.ardupilot.org/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/ 
for more details
=========================================================
EOF
        exit 1
    fi
}


autotest="../Tools/autotest"
[ -d "$autotest" ] || {
    # we are not running from one of the standard vehicle directories. Use 
    # the location of the sim_vehicle.sh script to find the path
    autotest=$(dirname $(readlink -e $0))
}

# modify build target based on copter frame type
case $FRAME in
    +|quad|quad-*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/copter_params.parm"
	;;
    X*)
	BUILD_TARGET="sitl"
        EXTRA_PARM="param set FRAME 1;"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/copter_params.parm"
	;;
    octa*)
	BUILD_TARGET="sitl-octa"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/copter_params.parm"
	;;
    tri*)
	BUILD_TARGET="sitl-tri"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/tri_params.parm"
	;;
    y6*)
	BUILD_TARGET="sitl-y6"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/y6_params.parm"
	;;
    firefly*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/firefly.parm"
	;;
    heli-dual)
        BUILD_TARGET="sitl-heli-dual"
        MODEL="heli-dual"
        ;;
    heli-compound)
        BUILD_TARGET="sitl-heli-compound"
        MODEL="heli-compound"
        ;;
    heli*)
	BUILD_TARGET="sitl-heli"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/Helicopter.parm"
	;;
    singlecopter*)
	BUILD_TARGET="sitl-single"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/SingleCopter.parm"
	;;
    coaxcopter*)
	BUILD_TARGET="sitl-coax"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/CoaxCopter.parm"
	;;
    IrisRos)
	BUILD_TARGET="sitl"
        DEFAULTS_PATH="$autotest/copter_params.parm"
	;;
    Gazebo)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/copter_params.parm"
	;;
    CRRCSim|last_letter*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
	;;
    jsbsim*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        check_jsbsim_version
        DEFAULTS_PATH="$autotest/ArduPlane.parm"
	;;
    quadplane-tilttri*)
	BUILD_TARGET="sitl-tri"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/quadplane-tilttri.parm"
	;;
    quadplane*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/quadplane.parm"
	;;
    plane-elevon*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/plane-elevons.parm"
	;;
    plane-vtail*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/plane-vtail.parm"
	;;
    plane*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/plane.parm"
	;;
    rover-skid)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/Rover-skid.parm"
	;;
    flightaxis*)
        MODEL="$FRAME"
        SITLRCIN=0
	;;
    *-heli)
	BUILD_TARGET="sitl-heli"
        MODEL="$FRAME"
        DEFAULTS_PATH="$autotest/Helicopter.parm"
	;;
    *)
        MODEL="$FRAME"
	;;
esac

if [ $DEBUG_BUILD == 1 ]; then
    BUILD_TARGET="$BUILD_TARGET-debug"
fi

if [ -n "$OVERRIDE_BUILD_TARGET" ]; then
    BUILD_TARGET="$OVERRIDE_BUILD_TARGET"
fi

VEHICLEDIR="$autotest/../../$VEHICLE"
[ -d "$VEHICLEDIR" ] || {
    VEHICLEDIR=$(dirname $(readlink -e $VEHICLEDIR))
}
pushd $VEHICLEDIR || {
    echo "Failed to change to vehicle directory for $VEHICLEDIR"
    usage
    exit 1
}
AUTOTEST=$autotest
export AUTOTEST
VEHICLEDIR=$(pwd)

if [ $NO_REBUILD == 0 ]; then
if [ $CLEAN_BUILD == 1 ]; then
    echo "Building clean"
    make clean
fi
echo "Building $BUILD_TARGET"
make $BUILD_TARGET -j$NUM_PROCS || {
    make clean
    make $BUILD_TARGET -j$NUM_PROCS || {
	echo >&2 "$0: Build failed"
	exit 1
    }
}
fi
popd

# get the location information
if [ -z $CUSTOM_LOCATION ]; then
    SIMHOME=$(cat $autotest/locations.txt | grep -i "^$LOCATION=" | head -1 | cut -d= -f2)
else
    SIMHOME=$CUSTOM_LOCATION
    LOCATION="Custom_Location"
fi

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
    make sitl-debug -j$NUM_PROCS || {
        make clean
        make sitl-debug -j$NUM_PROCS
    }
    TRACKER_INSTANCE=1
    TRACKER_UARTA="tcp:127.0.0.1:"$((5760+10*$TRACKER_INSTANCE))
    cmd="nice /tmp/AntennaTracker.build/AntennaTracker.elf -I1 --model=tracker --home=$TRACKER_HOME"
    $autotest/run_in_terminal_window.sh "AntennaTracker" $cmd || exit 1
    popd
fi

cmd="$VEHICLEDIR/$VEHICLE.elf -S -I$INSTANCE --home $SIMHOME"
if [ $WIPE_EEPROM == 1 ]; then
    cmd="$cmd -w"
fi

cmd="$cmd --model $MODEL --speedup=$SPEEDUP $EXTRA_ARGS"

if [ $USE_MAVLINK_GIMBAL == 1 ]; then
    echo "Using MAVLink gimbal"
    cmd="$cmd --gimbal"
fi

if [ -n "$DEFAULTS_PATH" ]; then
    echo "Using defaults from $DEFAULTS_PATH"
    cmd="$cmd --defaults $DEFAULTS_PATH"
fi

if [ $START_HIL == 0 ]; then
if [ $USE_VALGRIND == 1 ]; then
    echo "Using valgrind"
    $autotest/run_in_terminal_window.sh "ardupilot (valgrind)" valgrind $cmd || exit 1
elif [ $USE_GDB == 1 ]; then
    echo "Using gdb"
    tfile=$(mktemp)
    [ $USE_GDB_STOPPED == 0 ] && {
        if [ -n "$BREAKPOINT" ]; then
            echo "b $BREAKPOINT" >> $tfile
        fi
        echo r >> $tfile
    }
    $autotest/run_in_terminal_window.sh "ardupilot (gdb)" gdb -x $tfile --args $cmd || exit 1
else
    $autotest/run_in_terminal_window.sh "ardupilot" $cmd || exit 1
fi
fi

if [ $START_HIL == 1 ]; then
    $autotest/run_in_terminal_window.sh "JSBSim" $autotest/jsb_sim/runsim.py --home $SIMHOME --speedup=$SPEEDUP || exit 1
fi

trap kill_tasks SIGINT

# mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 
options=""
if [ $START_HIL == 0 ]; then
    options="--master $MAVLINK_PORT"
    if [ $SITLRCIN == 1 ]; then
        options="$options --sitl $SIMOUT_PORT"
    fi
fi

# If running inside of a vagrant guest, then we probably want to forward our mavlink out to the containing host OS
if [ $USER == "vagrant" ]; then
options="$options --out 10.0.2.2:14550"
fi
options="$options --out 127.0.0.1:14550 --out 127.0.0.1:14551"
extra_cmd=""
if [ $START_ANTENNA_TRACKER == 1 ]; then
    options="$options --load-module=tracker"
    extra_cmd="$extra_cmd module load map; tracker set port $TRACKER_UARTA; tracker start; tracker arm"
fi
if [ $START_HIL == 1 ]; then
    options="$options --load-module=HIL"
fi
if [ $USE_MAVLINK_GIMBAL == 1 ]; then
    options="$options --load-module=gimbal"
fi
if [ $DELAY_START != 0 ]; then
  sleep $DELAY_START
fi

if [ $MAVLINK_PROTOCOL_VERSION == 2 ]; then
    options="$options --mav20"
fi

if [ -f /usr/bin/cygstart ]; then
    cygstart -w "/cygdrive/c/Program Files (x86)/MAVProxy/mavproxy.exe" $options --cmd="$extra_cmd" $*
else
    mavproxy.py $options --cmd="$extra_cmd" $*
fi
if [ $START_HIL == 0 ]; then
kill_tasks
fi

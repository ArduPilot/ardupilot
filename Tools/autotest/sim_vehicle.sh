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
TRACKER_ARGS=""
EXTERNAL_SIM=0
MODEL=""
BREAKPOINT=""
OVERRIDE_BUILD_TARGET=""
START_ROS_FROM_APM=0
ROS_LAUNCH_FILE=""

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
    -A               pass arguments to antenna tracker
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
    -e               use external simulator
    -S SPEEDUP       set simulation speedup (1 for wall clock time)
    -r LAUNCH_FILE   order Ardupilot to launch ROS/Gazebo, calling the ROS .launch start file

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
while getopts ":I:VgGcj:TA:t:L:l:v:hwf:RNHer:MS:DB:b:" opt; do
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
    D)
      DEBUG_BUILD=1
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
    e)
      EXTERNAL_SIM=1
      ;;
    b)
      OVERRIDE_BUILD_TARGET="$OPTARG"
      ;;
    r)
      ROS_LAUNCH_FILE=$OPTARG
      START_ROS_FROM_APM=1
      echo "ROS/Gazebo will be started by Ardupilot"
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
SIMIN_PORT="127.0.0.1:"$((5502+10*$INSTANCE))
SIMOUT_PORT="127.0.0.1:"$((5501+10*$INSTANCE))
FG_PORT="127.0.0.1:"$((5503+10*$INSTANCE))
# Output port from ROS' MAVROS plugin:
MAVLINK_ROS_PORT="127.0.0.1:"$((14550+10*$INSTANCE))

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
        RED='\033[0;31m'
        NC='\033[0m' # No Color
        echo -en "$RED"
        cat <<EOF
=========================================================
You need the latest ArduPilot version of JSBSim installed
and in your \$PATH

Please get it from git://github.com/tridge/jsbsim.git
See 
  http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/ 
for more details
=========================================================
EOF
        echo -en "$NC"
        exit 1
    fi
}

# This function checks that the ROS is installed on the system.
# If ROS is missing, displays an error message and aborts the script.
check_ros_version()
{
    ros_version=$(rosversion -d)
    if [[ $ros_version = *"command not found"* ]]
    then
        # ROS was not found: displays the error message and aborts
        RED='\033[0;31m'
        NC='\033[0m' # No Color
        echo -en "$RED"
        cat <<EOF
=========================================================
ROS was not found on the system !

You can find instructions on how to download and set it up on:
https://github.com/AurelienRoy/ardupilot_sitl_ros_tutorial

See also
  http://dev.ardupilot.com/wiki/using-rosgazebo-simulator-with-sitl/
for more details
=========================================================
EOF
        echo -en "$NC"
        exit 1
    fi
    echo "ROS $ros_version was found"
}

# This function checks that the ROS/Gazebo plugin named "ardupilot_sitl_gazebo_plugin"
# is present, and will save the path to its directory in the variable 'ardu_ros_pkg_path'.
# If the plugin is missing, displays an error message and aborts the script.
get_path_ardu_ros_plugin()
{
    ardu_ros_pkg_name="ardupilot_sitl_gazebo_plugin"
    # Searches the plugin name in the list of plugins
    ardu_ros_pkg=$(rospack list | grep -i "^$ardu_ros_pkg_name")
    if [ -z "$ardu_ros_pkg" ]
    then
        # the plugin was not found: displays the error message and aborts
        RED='\033[0;31m'
        NC='\033[0m' # No Color
        echo -en "$RED"
        cat <<EOF
=========================================================
The ardupilot Gazebo plugin was not found !
Expected plugin name: $ardu_ros_pkg_name

You can find instructions on how to download  and set it up on:
https://github.com/AurelienRoy/ardupilot_sitl_ros_tutorial

See also
  http://dev.ardupilot.com/wiki/using-rosgazebo-simulator-with-sitl/
for more details
=========================================================
EOF
        echo -en "$NC"
        exit 1
    fi
    # The plugin was found: extracts its directory path
    ARDU_ROS_PKG_PATH=$(echo $ardu_ros_pkg | cut -d ' ' -f2)
    echo "Ardupilot Gazebo plugin found on path:"
    echo "  $ARDU_ROS_PKG_PATH"
}




# modify build target based on copter frame type
case $FRAME in
    +|quad)
	BUILD_TARGET="sitl"
        MODEL="+"
	;;
    X)
	BUILD_TARGET="sitl"
        EXTRA_PARM="param set FRAME 1;"
        MODEL="X"
	;;
    octa*)
	BUILD_TARGET="sitl-octa"
        MODEL="$FRAME"
	;;
    heli*)
	BUILD_TARGET="sitl-heli"
        MODEL="$FRAME"
	;;
    heli-dual)
    BUILD_TARGET="sitl-heli-dual"
        EXTRA_SIM="$EXTRA_SIM --frame=heli-dual"
        MODEL="heli-dual"
    ;;
    heli-compound)
    BUILD_TARGET="sitl-heli-compound"
        EXTRA_SIM="$EXTRA_SIM --frame=heli-compound"
        MODEL="heli-compound"
    ;;
    IrisRos)
	BUILD_TARGET="sitl"
	;;
    Gazebo)
	BUILD_TARGET="sitl"
        EXTRA_SIM="$EXTRA_SIM --frame=Gazebo"
        MODEL="$FRAME"
        check_ros_version
        get_path_ardu_ros_plugin
	;;
    CRRCSim-heli)
	BUILD_TARGET="sitl-heli"
        MODEL="$FRAME"
	;;
    CRRCSim|last_letter*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
	;;
    jsbsim*)
	BUILD_TARGET="sitl"
        MODEL="$FRAME"
        check_jsbsim_version
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

autotest="../Tools/autotest"
[ -d "$autotest" ] || {
    # we are not running from one of the standard vehicle directories. Use 
    # the location of the sim_vehicle.sh script to find the path
    autotest=$(dirname $(readlink -e $0))
}
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
    SIMHOME=$(cat $autotest/locations.txt | grep -i "^$LOCATION=" | cut -d= -f2)
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

# For a Gazebo simulator, generates a GPS home reference .xacro file for the ROS GPS sensor
if [[ $FRAME == "Gazebo" ]]; then
    ROS_HOMELOC_FILE_PATH="$ARDU_ROS_PKG_PATH/urdf/gps_home_location.xacro"
    HOME_LATITUDE=$(echo $SIMHOME | cut -d, -f1)
    HOME_LONGITUDE=$(echo $SIMHOME | cut -d, -f2)
    HOME_ALTITUDE=$(echo $SIMHOME | cut -d, -f3)
    echo "Generated the file setting the GPS location of Gazebo map's center:"
    echo "  $ROS_HOMELOC_FILE_PATH"
    
    cat >$ROS_HOMELOC_FILE_PATH <<EOL
<?xml version="1.0"?>
<!-- File auto-generated by sim_vehicle.sh -->
<!-- Location at $LOCATION -->
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- The following GPS coordinates locate the center of Gazebo's map -->
  <xacro:property name="referenceLatitude" value="$HOME_LATITUDE" />     <!-- in degrees -->
  <xacro:property name="referenceLongitude" value="$HOME_LONGITUDE" />   <!-- in degrees -->
  <xacro:property name="referenceAltitude" value="$HOME_ALTITUDE" />     <!-- in meters above sea level -->
</robot>
EOL
fi

cmd="$VEHICLEDIR/$VEHICLE.elf -S -I$INSTANCE --home $SIMHOME"
if [ $WIPE_EEPROM == 1 ]; then
    cmd="$cmd -w"
fi

cmd="$cmd --model $MODEL --speedup=$SPEEDUP"

# Always pass on to Ardupilot's executable the path to the autotest directory,
# in case the simulator interface in Ardupilot needs it
cmd="$cmd --autotest-dir $autotest"

# For a Gazebo simulator, passes on to Ardupilot the name of the ROS .launch file to invoke
if [[ $FRAME == "Gazebo" && "$START_ROS_FROM_APM" == 1 ]]; then
    cmd="$cmd --ros-launch $ROS_LAUNCH_FILE"
fi


case $VEHICLE in
    ArduPlane)
        PARMS="ArduPlane.parm"
        ;;
    ArduCopter)
        PARMS="copter_params.parm"
        ;;
    APMrover2)
        PARMS="Rover.parm"
        ;;
    *)
        PARMS=""
        ;;
esac

if [ $USE_MAVLINK_GIMBAL == 1 ]; then
    echo "Using MAVLink gimbal"
    cmd="$cmd --gimbal"
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
    if [[ $FRAME != "Gazebo" ]]; then
        options="--master $MAVLINK_PORT --sitl $SIMOUT_PORT"
    else
        # special input ports settings for ROS/Gazebo simulations
        options="--master $MAVLINK_ROS_PORT"
    fi
fi

# If running inside of a vagrant guest, then we probably want to forward our mavlink out to the containing host OS
if [[ $FRAME == "Gazebo" ]]; then
    options="$options --out 127.0.0.1:14551"
    # Support Vagrant on Gazebo simulation ?
else
    if [ $USER == "vagrant" ]; then
        options="$options --out 10.0.2.2:14550"
    fi
    options="$options --out 127.0.0.1:14550 --out 127.0.0.1:14551"
fi


extra_cmd=""
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
if [ $USE_MAVLINK_GIMBAL == 1 ]; then
    options="$options --load-module=gimbal"
fi

# For a Gazebo simulator, looks for a map image of the Gazebo world, to pass it on to
# MavProxy as an overlay image
if [[ $FRAME == "Gazebo" ]]; then
    # Map images should be in the directory <ros_plugin>/worlds/<launch file name>
    MAVPROXY_MAP_OVERLAY_DIR="$ARDU_ROS_PKG_PATH/worlds/$ROS_LAUNCH_FILE"
    
    # List of image extensions supported by OpenCV (on which MavProxy rely to open an image file)
    MAP_IMG_PATTERN="-iname map_*.png"
    MAP_IMG_PATTERN="$MAP_IMG_PATTERN -or -iname map_*.jpg"
    MAP_IMG_PATTERN="$MAP_IMG_PATTERN -or -iname map_*.jpeg"
    MAP_IMG_PATTERN="$MAP_IMG_PATTERN -or -iname map_*.bmp"
    MAP_IMG_PATTERN="$MAP_IMG_PATTERN -or -iname map_*.dib"
    MAP_IMG_PATTERN="$MAP_IMG_PATTERN -or -iname map_*.gif"
    MAP_IMG_PATTERN="$MAP_IMG_PATTERN -or -iname map_*.tif"
    MAP_IMG_PATTERN="$MAP_IMG_PATTERN -or -iname map_*.tiff"
    
    # Finds the first file in the 'MAVPROXY_MAP_OVERLAY_DIR' directory that matches
    # the name "map_*.XXX" (case insensitive):
    MAVPROXY_MAP_IMG=$(find "$MAVPROXY_MAP_OVERLAY_DIR" -type f \( $MAP_IMG_PATTERN \) -print -quit)
    
    if [ -z "$MAVPROXY_MAP_IMG" ]; then
        # Displays a warning message
        ORANGE='\033[0;33m'
        NC='\033[0m' # No Color
        echo -en "$ORANGE"
        cat <<EOF
=========================================================
Could not find a MavProxy overlay map image for the world '$ROS_LAUNCH_FILE'.

If you wish to add one, place it in:
  $MAVPROXY_MAP_OVERLAY_DIR
The map image should be named: map_w<width>m_h<height>m.<png|jpg|bmp|...>
where <width> and <height> are the map size in meters.
  e.g.: map_w40m_h40m.jpg
=========================================================
EOF
    echo -en "$NC"
    else
        echo "MavProxy overlay map image found for the world '$ROS_LAUNCH_FILE':"
        echo "  $MAVPROXY_MAP_IMG"
        echo "it will be centered on lat=$HOME_LATITUDE lon=$HOME_LONGITUDE"
        
        # Re-uses 'HOME_LATITUDE' and 'HOME_LONGITUDE' that have been defined a bit above
        # in another 'Gazebo' special section
        extra_cmd="$extra_cmd map overlay $MAVPROXY_MAP_IMG $HOME_LATITUDE $HOME_LONGITUDE;"
    fi
fi

echo "Starting MavProxy : mavproxy.py $options --cmd=\"$extra_cmd\" $*"

if [ -f /usr/bin/cygstart ]; then
    cygstart -w "/cygdrive/c/Program Files (x86)/MAVProxy/mavproxy.exe" $options --cmd="$extra_cmd" $*
else
    mavproxy.py $options --cmd="$extra_cmd" $*
fi
if [ $START_HIL == 0 ]; then
kill_tasks
fi

#!/bin/bash

set -x

#
# Variables to configure SITL environment
#
CLEAN_BUILD=0
USE_VALGRIND=0
USE_GDB=0
USE_LLDB=0
OS=$(uname)

target=sitl
frame="+"

if [ $# -gt 0 ]; then
    case $1 in
	+|X|quad)
	    target="sitl"
	    frame="$1"
	    shift
	    ;;
	octa)
	    target="sitl-octa"
	    frame="$1"
	    shift
	    ;;
    esac
fi

echo "Building with target $target for frame $frame"

# 
# Kill any running instances
# 

if [ "$OS" = "Darwin" ]; then
  KILLALL="killall"
else
  KILLALL="killall -q"
fi

$KILLALL ArduCopter.elf
pkill -f sim_multicopter.py

#
# Locate tempdir where build files are dumped
#

if [ -z "$TMPDIR" ]; then
  TMPDIR="/tmp/"
fi

#
# Locate the autotest directory for each OS
#

if [ "$OS" = "Darwin" ]; then
  autotest=$(dirname $0)
else
  autotest=$(dirname $(readlink -e $0))
fi

#
# Clean and compile the target
#

pushd $autotest/../../ArduCopter

if [ $CLEAN_BUILD == 1 ]; then
  make clean
fi

make $target -j4 || {
    make clean
    make $target -j4
}

#
# Launch SITL's parts, with the ArduCopter binary either
# vanilla, or through useful debugging tools (gdb, valgrind, etc).
#

if [ "$OS" = "Darwin" ]; then
  cmd="${TMPDIR}ArduCopter.build/ArduCopter.elf"
else
  cmd="${TMPDIR}ArduCopter.build/ArduCopter.elf -I$INSTANCE"
fi

if [ $USE_VALGRIND == 1 ]; then
  echo "Using valgrind"
  $autotest/run_in_terminal_window.sh "ardupilot (valgrind)" valgrind $cmd || exit 1
elif [ $USE_GDB == 1 ]; then
  echo "Using gdb"
  tfile=$(mktemp)
  echo r > $tfile
  $autotest/run_in_terminal_window.sh "ardupilot (gdb)" gdb -x $tfile --args $cmd || exit 1
elif [ $USE_LLDB == 1 ]; then
    echo "Using lldb"
    sfile=$(mktemp lldb.XXXX)
#   echo "process handle SIGALRM -n false -p true" >> $sfile
    echo "target create $cmd" > $sfile
    echo "process launch" >> $sfile
    $autotest/run_in_terminal_window.sh "ardupilot (lldb)" lldb -s $sfile  || exit 1
else
  $autotest/run_in_terminal_window.sh "ardupilot" $cmd || exit 1
fi

#
# Launch the multicopter simulator
#

if [ "$OS" = "Darwin" ]; then
  SLEEPTIME=4
else
  SLEEPTIME=2
fi

sleep $SLEEPTIME
rm -f $tfile
$autotest/run_in_terminal_window.sh "sim_multicopter" "$autotest/pysim/sim_multicopter.py --frame=$frame --home=-35.362938,149.165085,584,180" || exit 1
popd
sleep $SLEEPTIME

#
# Launch the ground station
#

mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --quadcopter $* 

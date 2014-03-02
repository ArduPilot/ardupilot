#!/bin/bash

# home location lat, lon, alt, heading
SIMHOME="-35.363261,149.165230,584,353"

# check the instance number to allow for multiple copies of the sim running at once
INSTANCE=0
USE_VALGRIND=0
USE_GDB=0
CLEAN_BUILD=0

# parse options. Thanks to http://wiki.bash-hackers.org/howto/getopts_tutorial
while getopts ":I:" opt; do
  case $opt in
    I)
      INSTANCE=$OPTARG
      ;;
    V)
      USE_VALGRIND=1
      ;;
    G)
      USE_GDB=1
      ;;
    c)
      CLEAN_BUILD=1
      ;;
    \?)
      # allow other args to pass on to mavproxy
      break
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
  esac
done
shift $((OPTIND-1))

# kill existing copy if this is the '0' instance only
[ "$INSTANCE" -eq "0" ] && {
    killall -q JSBSim lt-JSBSim ArduPlane.elf
    pkill -f runsim.py
}


# setup ports for this instance
MAVLINK_PORT="tcp:127.0.0.1:"$((5760+10*$INSTANCE))
SIMIN_PORT="127.0.0.1:"$((5502+10*$INSTANCE))
SIMOUT_PORT="127.0.0.1:"$((5501+10*$INSTANCE))
FG_PORT="127.0.0.1:"$((5503+10*$INSTANCE))

set -e
set -x

autotest=$(dirname $(readlink -e $0))
pushd $autotest/../../ArduPlane
if [ $CLEAN_BUILD == 1 ]; then
    make clean
fi
make sitl -j4 || {
    make clean
    make sitl -j4
}

cmd="/tmp/ArduPlane.build/ArduPlane.elf -I$INSTANCE"

if [ $USE_VALGRIND == 1 ]; then
    $autotest/run_in_terminal_window.sh "ardupilot (valgrind)" valgrind -q $cmd || exit 1
elif [ $USE_GDB == 1 ]; then
    tfile=$(mktemp)
    echo r > $tfile
    $autotest/run_in_terminal_window.sh "ardupilot (gdb)" gdb -x $tfile --args $cmd || exit 1
else
    $autotest/run_in_terminal_window.sh "ardupilot" $cmd || exit 1
fi

sleep 2
rm -f $tfile
$autotest/run_in_terminal_window.sh "JSBSim" ../Tools/autotest/jsbsim/runsim.py --home=$SIMHOME --simin=$SIMIN_PORT --simout=$SIMOUT_PORT --fgout=$FG_PORT || exit 1
sleep 2
popd

# if you wanted to forward packets out a serial link for testing
# andropilot, then add --out /dev/serial/by-id/usb-FTDI* to your
# command line along with a baudrate. You might also like to add --map
# and --console
# for example:
# sim_arduplane.sh --out /dev/serial/by-id/usb-FTDI* --baudrate 57600 --map --console

# mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 
mavproxy.py --master $MAVLINK_PORT --sitl $SIMOUT_PORT --out 127.0.0.1:14550 --out 127.0.0.1:14551 $*

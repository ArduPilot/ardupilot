#!/bin/bash

# home location lat, lon, alt, heading
SIMHOME="-35.363261,149.165230,584,353"

# check the instance number to allow for multiple copies of the sim running at once
INSTANCE=0

# Try to run a command in an appropriate type of terminal window
# depending on whats available
# Sigh: theres no common way of handling command line args :-(
function run_in_terminal_window()
{
    if [ -x /usr/bin/konsole ]; then
	 /usr/bin/konsole --hold -e $*
    elif [ -x /usr/bin/gnome-terminal ]; then
	 /usr/bin/gnome-terminal -e "$*"
    elif [ -x /usr/bin/xterm ]; then
	 /usr/bin/xterm -hold -e $* &
    else
	# out of options: run in the background
	$* &
    fi
}

# parse options. Thanks to http://wiki.bash-hackers.org/howto/getopts_tutorial
while getopts ":I:" opt; do
  case $opt in
    I)
      INSTANCE=$OPTARG
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
    killall -q AntennaTracker.elf
}


# setup ports for this instance
MAVLINK_PORT="tcp:127.0.0.1:"$((5760+10*$INSTANCE))

set -e
set -x

autotest=$(dirname $(readlink -e $0))
pushd $autotest/../../Tools/AntennaTracker
make clean sitl

tfile=$(mktemp)
echo r > $tfile
#run_in_terminal_window gdb -x $tfile --args /tmp/AntennaTracker.build/AntennaTracker.elf
#run_in_terminal_window /tmp/AntennaTracker.build/AntennaTracker.elf -I$INSTANCE
run_in_terminal_window /tmp/AntennaTracker.build/AntennaTracker.elf -I$INSTANCE

#run_in_terminal_window valgrind -q /tmp/AntennaTracker.build/AntennaTracker.elf
sleep 2
rm -f $tfile
popd

mavproxy.py --master $MAVLINK_PORT $*

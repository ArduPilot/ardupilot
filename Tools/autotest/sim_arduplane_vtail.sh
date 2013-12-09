#!/bin/bash

killall -q JSBSim
killall -q ArduPlane.elf
pkill -f runsim.py

set -e
set -x

autotest=$(dirname $(readlink -e $0))
pushd $autotest/../../ArduPlane
make clean sitl

tfile=$(mktemp)
echo r > $tfile
#gnome-terminal -e "gdb -x $tfile --args /tmp/ArduPlane.build/ArduPlane.elf"
gnome-terminal -e /tmp/ArduPlane.build/ArduPlane.elf
#gnome-terminal -e "valgrind -q /tmp/ArduPlane.build/ArduPlane.elf"
sleep 2
rm -f $tfile
gnome-terminal -e '../Tools/autotest/jsbsim/runsim.py --home=-35.362938,149.165085,584,270 --vtail'
sleep 2
popd

# if you wanted to forward packets out a serial link for testing
# andropilot, then add --out /dev/serial/by-id/usb-FTDI* to your
# command line along with a baudrate. You might also like to add --map
# and --console
# for example:
# sim_arduplane.sh --out /dev/serial/by-id/usb-FTDI* --baudrate 57600 --map --console

mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 $*


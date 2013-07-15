#!/bin/bash

killall -q ArduCopter.elf
pkill -f sim_multicopter.py

set -e
set -x

target=sitl
frame="X"

echo "Building with target $target for frame $frame"

autotest=$(dirname $(readlink -e $0))
pushd $autotest/../../ArduCopter
make clean $target

tfile=$(mktemp)
echo r > $tfile
#gnome-terminal -e "gdb -x $tfile --args /tmp/ArduCopter.build/ArduCopter.elf"
#gnome-terminal -e /tmp/ArduCopter.build/ArduCopter.elf
gnome-terminal -e "valgrind -q /tmp/ArduCopter.build/ArduCopter.elf"
sleep 2
rm -f $tfile
gnome-terminal -e "../Tools/autotest/pysim/sim_multicopter.py --frame=$frame --home=-35.362938,149.165085,584,270"
sleep 2
popd
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 192.168.1.12:14550 --console --map --aircraft test --quadcopter $*

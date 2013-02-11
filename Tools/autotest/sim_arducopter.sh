#!/bin/bash

killall -q ArduCopter.elf
pkill -f sim_multicopter.py

set -e
set -x

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

autotest=$(dirname $(readlink -e $0))
pushd $autotest/../../ArduCopter
make clean $target

tfile=$(mktemp)
echo r > $tfile
#gnome-terminal -e "gdb -x $tfile --args /tmp/ArduCopter.build/ArduCopter.elf"
gnome-terminal -e /tmp/ArduCopter.build/ArduCopter.elf
#gnome-terminal -e "valgrind -q /tmp/ArduCopter.build/ArduCopter.elf"
sleep 2
rm -f $tfile
gnome-terminal -e "../Tools/autotest/pysim/sim_multicopter.py --frame=$frame --home=-35.362938,149.165085,584,270"
sleep 2
popd
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --quadcopter $*

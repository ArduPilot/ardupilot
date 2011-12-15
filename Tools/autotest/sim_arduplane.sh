#!/bin/bash

set -e
set -x

autotest=$(dirname $(readlink -e $0))
pushd $autotest/../../ArduPlane
make clean sitl

tfile=$(tempfile)
echo r > $tfile
gnome-terminal -e "gdb -x $tfile --args /tmp/ArduPlane.build/ArduPlane.elf"
sleep 2
rm -f $tfile
gnome-terminal -e '../Tools/autotest/jsbsim/runsim.py --home=-35.362938,149.165085,584,270 --wind=5,180,0.2'
sleep 2
popd
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551

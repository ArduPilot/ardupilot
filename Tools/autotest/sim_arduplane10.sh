#!/bin/bash

set -e
set -x

: ${APMTERM=xterm}

autotest=$(dirname $(readlink -e $0))
pushd $autotest/../../ArduPlane
make clean sitl-mavlink10

tfile=$(mktemp)
echo r > $tfile
#$APMTERM -e "gdb -x $tfile --args /tmp/ArduPlane.build/ArduPlane.elf" &
$APMTERM -e /tmp/ArduPlane.build/ArduPlane.elf &
#$APMTERM -e "valgrind -q /tmp/ArduPlane.build/ArduPlane.elf" &
sleep 2
rm -f $tfile
$APMTERM -e '../Tools/autotest/jsbsim/runsim.py --home=-35.362938,149.165085,584,270' &
sleep 2
popd
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --mav10

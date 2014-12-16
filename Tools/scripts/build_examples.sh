#!/bin/bash
# useful script to test the build of all example code
# This helps when doing large merges
# Andrew Tridgell, November 2012

set -e
set -x

targets="clean apm2"

[ $# -gt 0 ] && {
    targets="$*"
}

export PATH=/usr/lib/ccache:$PATH

TESTS=$(find libraries -name 'Makefile' | grep -v FLYMAPLE | xargs -i dirname '{}')

for b in $TESTS; do
    echo "TESTING $b"
    pushd $b
    if [ -r nobuild.txt ]; then
	echo "Skipping build of $b"
    else
	for t in $targets; do
	    make -j4 $t
	done
    fi
    popd
done

echo "Building some examples for px4-v2"
test -d ../PX4Firmware && {
  for d in libraries/AP_GPS/examples/GPS_AUTO_test libraries/AP_InertialSensor/examples/INS_generic libraries/AP_Compass/examples/AP_Compass_test libraries/AP_Baro/examples/BARO_generic libraries/AP_AHRS/examples/AHRS_Test; do
      echo "Building $d for px4-v2"
      pushd $d
      make px4-v2
      popd
  done
}

echo "All examples built OK"
exit 0

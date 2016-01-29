#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. config.mk

set -e
set -x

export BUILDROOT="/tmp/all.build"
rm -rf $BUILDROOT

echo "Testing ArduPlane build"
pushd ArduPlane
for b in sitl linux; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing ArduCopter build"
pushd ArduCopter
for b in sitl linux; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing APMRover build"
pushd APMrover2
for b in sitl linux; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing AntennaTracker build"
pushd AntennaTracker
for b in sitl; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing build of examples"

examples="Tools/CPUInfo"
for d in $examples; do
    pushd $d
    make clean
    make sitl -j4
    popd
done

test -d ../libmaple && {
echo "Testing flymaple build"
for d in APMrover2; do
    pushd $d
    make clean
    make flymaple -j4
    popd
done
}

pushd Tools/Replay
make clean
make linux -j4
popd

test -n "$PX4_ROOT" && test -d "$PX4_ROOT" && {
    ./Tools/scripts/build_all_px4.sh
}

test -n "$VRBRAIN_ROOT" && test -d "$VRBRAIN_ROOT" && {
    ./Tools/scripts/build_all_vrbrain.sh
}

exit 0

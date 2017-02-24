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

echo "Testing ArduSub build"
pushd ArduSub
for b in sitl linux; do
    pwd
    make clean
    make $b -j4
done
popd

pushd Tools/Replay
make clean
make
popd

test -n "$PX4_ROOT" && test -d "$PX4_ROOT" && {
    ./Tools/scripts/build_all_px4.sh
}

test -n "$VRBRAIN_ROOT" && test -d "$VRBRAIN_ROOT" && {
    ./Tools/scripts/build_all_vrbrain.sh
}

exit 0

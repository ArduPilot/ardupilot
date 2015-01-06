#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. config.mk

set -e
set -x

echo "Testing ArduPlane build"
pushd ArduPlane
make configure
for b in all apm2 sitl linux; do
    pwd
    make clean
    make $b -j4
done
popd

for d in ArduCopter APMrover2 ArduPlane AntennaTracker; do
    pushd $d
    make clean
    make sitl -j4
    make clean
    make linux -j4
    make clean
    make px4-cleandep
    make px4-v2
    popd
done

pushd Tools/Replay
make clean
make linux -j4
popd

exit 0

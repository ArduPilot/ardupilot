#!/usr/bin/env bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

set -e
set -x

export BUILDROOT="/tmp/all.build"
rm -rf $BUILDROOT

BOARDS="sitl linux Pixhawk1"

for b in $BOARDS; do
    echo "Testing $b build"
    ./waf configure --board $b
    ./waf clean
    ./waf
done

echo "Building Replay"
pushd Tools/Replay
make clean
make
popd

echo "Testing configure all"
./Tools/scripts/configure_all.py

exit 0

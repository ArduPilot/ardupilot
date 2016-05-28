#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. config.mk

set -e
set -x

echo "Testing ArduPlane build"
pushd ArduCopter
make configure
for b in all apm2 sitl; do
    pwd
    make clean
    make $b -j4
done
popd

for d in ArduCopter; do
    pushd $d
    make clean
    make sitl -j4
    make clean
    make px4-cleandep
    make px4-v2
    popd
done

exit 0

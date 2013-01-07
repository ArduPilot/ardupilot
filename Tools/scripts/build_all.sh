#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

set -e
set -x

echo "Testing ArduPlane build"
pushd ArduPlane
for b in all apm2 apm2beta apm1-hil apm1-hilsensors apm2-hil apm2-hilsensors sitl sitl-mount sitl-newcontrollers; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing ArduCopter build"
pushd ArduCopter
for b in all apm2 apm2beta apm1-hil apm2-hil sitl heli dmp; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing APMRover build"
pushd APMrover2
for b in all apm2 sitl; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing build of examples"

examples="Tools/VARTest Tools/CPUInfo"
for d in $examples; do
    pushd $d
    make clean
    make -j4
    popd
done

exit 0

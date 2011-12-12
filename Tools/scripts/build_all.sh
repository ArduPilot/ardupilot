#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

set -e
set -x

pushd ArduPlane
for b in all apm2 apm2beta hil hilsensors mavlink10 sitl; do
    pwd
    make clean
    make $b
done
popd

pushd ArduCopter
for b in all apm2 apm2beta hil sitl; do
    pwd
    make clean
    make $b
done
popd

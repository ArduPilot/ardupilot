#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

set -e
set -x

pushd ArduPlane
for b in all apm2 hil hilsensors; do
    pwd
    make clean
    make $b
done
make -f ../libraries/Desktop/Makefile.desktop clean all
popd

pushd ArduCopter
for b in all apm2 hil; do
    pwd
    make clean
    make $b
done
make -f ../libraries/Desktop/Makefile.desktop clean all
popd

#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. config.mk

set -e
set -x

. ~/.profile

travis_build_type_or_empty() {
    if [ -z "$TRAVIS_BUILD_TYPE" ] || [ "$TRAVIS_BUILD_TYPE" = "$1" ]; then
        return 0
    fi
    return 1
}

echo "Testing ArduPlane build"
pushd ArduPlane
for b in apm2; do
    pwd
    make clean
    make $b -j4
done
popd

for d in ArduCopter APMrover2 ArduPlane AntennaTracker; do
    if ! travis_build_type_or_empty "$d"; then
        continue
    fi
    pushd $d
    make clean
    make navio -j2
    make clean
    make sitl -j4
    make clean
    make linux -j2
    make clean
    make px4-cleandep
    make px4-v2
    popd
done

if travis_build_type_or_empty "Replay"; then
    pushd Tools/Replay
    make clean
    make linux -j4
    popd
fi

exit 0

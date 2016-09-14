#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. config.mk

set -e
set -x

. ~/.profile

# If TRAVIS_BUILD_TARGET is not set, default to all of them
if [ -z "$TRAVIS_BUILD_TARGET" ]; then
    TRAVIS_BUILD_TARGET="sitl apm2"
fi

declare -A build_platforms
declare -A build_concurrency
declare -A build_extra_clean

build_platforms=(  ["ArduPlane"]="apm2 sitl"
                   ["ArduCopter"]=" sitl "
                   ["APMrover2"]="apm2 sitl"
                   ["AntennaTracker"]="apm2"
                   ["Tools/Replay"]="linux")

build_concurrency=(["apm2"]="-j2")

echo "Targets: $TRAVIS_BUILD_TARGET"
for t in $TRAVIS_BUILD_TARGET; do
    for v in ${!build_platforms[@]}; do
        if [[ ${build_platforms[$v]} != *$t* ]]; then
            continue
        fi
        echo "Building $v for ${t}..."

        pushd $v
        make clean

        make $t ${build_concurrency[$t]}
        popd
    done
done

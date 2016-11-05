#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

set -ex

. ~/.profile

# CXX and CC are exported by default by travis
unset CXX CC

export BUILDROOT=/tmp/travis.build.$$
rm -rf $BUILDROOT

# If CI_BUILD_TARGET is not set, default to all of them
if [ -z "$CI_BUILD_TARGET" ]; then
    CI_BUILD_TARGET="sitl apm1 apm2"
fi

declare -A build_platforms
declare -A build_concurrency

build_platforms=(  ["ArduPlane"]="apm1 apm2 sitl"
                   ["ArduCopter"]="sitl"
                   ["APMrover2"]="apm1 apm2 sitl"
                   ["AntennaTracker"]="apm1 apm2 sitl"
                   ["Tools/Replay"]="linux")

build_concurrency=(["apm2"]="-j2"
                   ["apm1"]="-j2"
                   ["sitl"]="-j2"
                   ["linux"]="-j2")


echo "Targets: $CI_BUILD_TARGET"
for t in $CI_BUILD_TARGET; do
    echo "Starting make based build for target ${t}..."
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

echo build OK
exit 0
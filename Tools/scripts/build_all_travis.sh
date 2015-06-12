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

declare -A build_platforms
declare -A build_concurrency
declare -A build_extra_clean

build_platforms=(  ["ArduPlane"]="apm2 navio sitl linux px4-v2"
                   ["ArduCopter"]="navio sitl linux px4-v2"
                   ["APMrover2"]="navio sitl linux px4-v2"
                   ["AntennaTracker"]="navio sitl linux px4-v2"
                   ["Tools/Replay"]="linux")

build_concurrency=(["apm2"]="-j4"
                   ["navio"]="-j2"
                   ["sitl"]="-j4"
                   ["linux"]="-j2"
                   ["px4-v2"]="")

build_extra_clean=(["px4-v2"]="make px4-cleandep")

for d in "${!build_platforms[@]}"; do
    if ! travis_build_type_or_empty "$d"; then
        continue
    fi

    pushd $d
    for p in ${build_platforms["$d"]}; do
        make clean
        if [ ${build_extra_clean[$p]+_} ]; then
            ${build_extra_clean[$p]}
        fi

        make $p ${build_concurrency[$p]}
    done
    popd
done

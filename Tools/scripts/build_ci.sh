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
    CI_BUILD_TARGET="sitl linux navio raspilot minlure bebop px4-v2 px4-v4"
fi

declare -A build_platforms
declare -A build_concurrency
declare -A build_extra_clean
declare -A waf_supported_boards

build_platforms=(  ["ArduPlane"]="navio raspilot minlure bebop sitl linux px4-v2"
                   ["ArduCopter"]="navio raspilot minlure bebop sitl linux px4-v2 px4-v4"
                   ["APMrover2"]="navio raspilot minlure bebop sitl linux px4-v2"
                   ["AntennaTracker"]="navio raspilot minlure bebop sitl linux px4-v2"
                   ["Tools/Replay"]="linux")

build_concurrency=(["navio"]="-j2"
                   ["raspilot"]="-j2"
                   ["minlure"]="-j2"
                   ["bebop"]="-j2"
                   ["sitl"]="-j2"
                   ["linux"]="-j2"
                   ["px4-v2"]=""
                   ["px4-v4"]="")

build_extra_clean=(["px4-v2"]="make px4-cleandep")

waf=modules/waf/waf-light

# get list of boards supported by the waf build
for board in $($waf list_boards | head -n1); do waf_supported_boards[$board]=1; done

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
        if [ ${build_extra_clean[$t]+_} ]; then
            ${build_extra_clean[$t]}
        fi

        make $t ${build_concurrency[$t]}
        popd
    done

    if [[ -n ${waf_supported_boards[$t]} ]]; then
        echo "Starting waf build for board ${t}..."
        $waf configure --board $t
        $waf clean
        $waf ${build_concurrency[$t]} all
        if [[ $t == linux ]]; then
            $waf check
        fi
    fi
done

echo build OK
exit 0

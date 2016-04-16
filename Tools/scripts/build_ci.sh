#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

set -ex

. ~/.profile

# CXX and CC are exported by default by travis
c_compiler=${CC:-gcc}
cxx_compiler=${CXX:-g++}
unset CXX CC

export BUILDROOT=/tmp/ci.build
rm -rf $BUILDROOT
export GIT_VERSION="ci_test"
export NUTTX_GIT_VERSION="ci_test"
export PX4_GIT_VERSION="ci_test"
export CCACHE_SLOPPINESS="include_file_ctime,include_file_mtime"

if [[ "$cxx_compiler" == "clang++" ]]; then
  export CCACHE_CPP2="true"
fi

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

# special case for SITL testing in CI
if [ "$CI_BUILD_TARGET" = "sitltest" ]; then
    echo "Installing pymavlink"
    git submodule init
    git submodule update
    (cd modules/mavlink/pymavlink && python setup.py build install --user)
    unset BUILDROOT
    echo "Running SITL QuadCopter test"
    Tools/autotest/autotest.py -j2 build.ArduCopter fly.ArduCopter
    echo "Running SITL QuadPlane test"
    Tools/autotest/autotest.py -j2 build.ArduPlane fly.QuadPlane
    exit 0
fi

waf=modules/waf/waf-light

# get list of boards supported by the waf build
for board in $($waf list_boards | head -n1); do waf_supported_boards[$board]=1; done

echo "Temporarily disabling px4 waf builds (broken in px4 merge)"
waf_supported_boards[px4-v1]=""
waf_supported_boards[px4-v2]=""
waf_supported_boards[px4-v4]=""

echo "Targets: $CI_BUILD_TARGET"
for t in $CI_BUILD_TARGET; do
    # skip make-based build for clang
    if [[ "$cxx_compiler" != "clang++" ]]; then
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
    fi

    if [[ -n ${waf_supported_boards[$t]} ]]; then
        echo "Starting waf build for board ${t}..."
        $waf configure --board $t --enable-benchmarks --check-c-compiler="$c_compiler" --check-cxx-compiler="$cxx_compiler"
        $waf clean
        $waf ${build_concurrency[$t]} all
        if [[ $t == linux ]]; then
            $waf check
        fi
    fi
done

echo build OK
exit 0

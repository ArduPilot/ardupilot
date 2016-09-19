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

# If CI_BUILD_TARGET is not set, build 3 different ones
if [ -z "$CI_BUILD_TARGET" ]; then
    CI_BUILD_TARGET="sitl linux px4-v2"
fi

if [[ "$CI_BUILD_TARGET" == *"px4"* ]]; then
    export CCACHE_MAXSIZE="1500M"
elif [[ "$CI_BUILD_TARGET" == "sitltest" ]]; then
    export CCACHE_MAXSIZE="300M"
else
    export CCACHE_MAXSIZE="1000M"
fi

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

declare -A waf_supported_boards

waf=modules/waf/waf-light

# get list of boards supported by the waf build
for board in $($waf list_boards | head -n1); do waf_supported_boards[$board]=1; done

function get_time {
    date -u "+%s"
}

echo "Targets: $CI_BUILD_TARGET"
for t in $CI_BUILD_TARGET; do
    # only do make-based builds for GCC when target is PX4 or when launched by a scheduled job
    if [[ "$cxx_compiler" != "clang++" && ( $t == "px4"* || -n ${CI_CRON_JOB+1} ) ]]; then
        echo "Starting make based build for target ${t}..."
        for v in "ArduPlane" "ArduCopter" "APMrover2" "AntennaTracker"; do
            echo "Building $v for ${t}..."

            pushd $v
            make clean
            if [[ $t == "px4"* ]]; then
                make px4-cleandep
            fi

            start_time=$(get_time)
            make $t -j2
            diff_time=$(($(get_time)-$start_time))
            echo -e "\033[32m'make' finished successfully (${diff_time}s)\033[0m"
            ccache -s && ccache -z
            popd
        done

        if [[ $t == linux ]]; then
            echo "Building Replay for ${t}..."

            pushd "Tools/Replay"
            make clean
            make -j2
            popd
        fi
    fi

    if [[ -n ${waf_supported_boards[$t]} && -z ${CI_CRON_JOB+1} ]]; then
        echo "Starting waf build for board ${t}..."
        $waf configure --board $t --enable-benchmarks --check-c-compiler="$c_compiler" --check-cxx-compiler="$cxx_compiler"
        $waf clean
        $waf all
        ccache -s && ccache -z

        if [[ $t == linux ]]; then
            $waf check
        fi
    fi
done

echo build OK
exit 0

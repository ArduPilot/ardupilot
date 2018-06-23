#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. ~/.profile

set -ex

# CXX and CC are exported by default by travis
c_compiler=${CC:-gcc}
cxx_compiler=${CXX:-g++}
unset CXX CC

export BUILDROOT=/tmp/ci.build
rm -rf $BUILDROOT
export GIT_VERSION="ci_test"
export NUTTX_GIT_VERSION="ci_test"
export PX4_GIT_VERSION="ci_test"
export CHIBIOS_GIT_VERSION="ci_test"
export CCACHE_SLOPPINESS="include_file_ctime,include_file_mtime"

# If CI_BUILD_TARGET is not set, build 3 different ones
if [ -z "$CI_BUILD_TARGET" ]; then
    CI_BUILD_TARGET="sitl linux px4-v2"
fi

if [[ "$CI_BUILD_TARGET" == *"px4"* ]]; then
    export CCACHE_MAXSIZE="250M"
elif [[ "$CI_BUILD_TARGET" == "sitltest" ]]; then
    export CCACHE_MAXSIZE="20M"
else
    export CCACHE_MAXSIZE="150M"
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
    # special case for SITL testing in CI
    if [ $t == "sitltest-copter" ]; then
        echo "Installing pymavlink"
        git submodule init
        git submodule update
        (cd modules/mavlink/pymavlink && python setup.py build install --user)
        unset BUILDROOT
        echo "Running SITL QuadCopter test"
        Tools/autotest/autotest.py build.ArduCopter fly.ArduCopter
        ccache -s && ccache -z
        continue
    fi
    if [ $t == "sitltest-plane" ]; then
        echo "Installing pymavlink"
        git submodule init
        git submodule update
        (cd modules/mavlink/pymavlink && python setup.py build install --user)
        unset BUILDROOT
        echo "Running SITL Plane test"
        Tools/autotest/autotest.py build.ArduPlane fly.ArduPlane
        ccache -s && ccache -z
        continue
    fi
    if [ $t == "sitltest-quadplane" ]; then
        echo "Installing pymavlink"
        git submodule init
        git submodule update
        (cd modules/mavlink/pymavlink && python setup.py build install --user)
        unset BUILDROOT
        echo "Running SITL QuadPlane test"
        Tools/autotest/autotest.py build.ArduPlane fly.QuadPlane
        ccache -s && ccache -z
        continue
    fi
    if [ $t == "sitltest-rover" ]; then
        echo "Installing pymavlink"
        git submodule init
        git submodule update
        (cd modules/mavlink/pymavlink && python setup.py build install --user)
        unset BUILDROOT
        echo "Running SITL Rover test"
        Tools/autotest/autotest.py build.APMrover2 drive.APMrover2
        ccache -s && ccache -z
        continue
    fi

    if [ $t == "revo-bootloader" ]; then
        echo "Building revo bootloader"
        $waf configure --board revo-mini --bootloader
        $waf clean
        $waf bootloader
        continue
    fi

    if [ $t == "revo-mini" ]; then
        # save some time by only building one target for revo-mini
        echo "Building revo-mini"
        $waf configure --board revo-mini
        $waf clean
        $waf plane
        continue
    fi
    
    # only do make-based builds for GCC, when target is PX4-v3 or build is launched by a scheduled job and target is a PX4 board or SITL
    if [[ "$cxx_compiler" != "clang++" && ($t == "px4-v3" || (-n ${CI_CRON_JOB+1} && ($t == "px4"* || $t == "sitl"))) ]]; then
        echo "Starting make based build for target ${t}..."
        for v in "ArduPlane" "ArduCopter" "APMrover2" "ArduSub" "AntennaTracker"; do
            echo "Building $v for ${t}..."

            pushd $v
            make clean
            if [[ $t == "px4"* ]]; then
                make px4-cleandep
            fi

            start_time=$(get_time)
            CCACHE_DISABLE="true" make $t -j$(nproc)
            diff_time=$(($(get_time)-$start_time))
            echo -e "\033[32m'make' finished successfully (${diff_time}s)\033[0m"
            popd
        done
    fi

    if [[ -n ${waf_supported_boards[$t]} && -z ${CI_CRON_JOB+1} ]]; then
        echo "Starting waf build for board ${t}..."
        $waf configure --board $t \
                --enable-benchmarks \
                --enable-header-checks \
                --check-c-compiler="$c_compiler" \
                --check-cxx-compiler="$cxx_compiler"
        $waf clean
        $waf all
        ccache -s && ccache -z

        if [[ $t == linux ]]; then
            $waf check
        fi
    fi
done

python Tools/autotest/param_metadata/param_parse.py --no-emit --vehicle APMrover2
python Tools/autotest/param_metadata/param_parse.py --no-emit --vehicle AntennaTracker
python Tools/autotest/param_metadata/param_parse.py --no-emit --vehicle ArduCopter
python Tools/autotest/param_metadata/param_parse.py --no-emit --vehicle ArduPlane
python Tools/autotest/param_metadata/param_parse.py --no-emit --vehicle ArduSub

echo build OK
exit 0

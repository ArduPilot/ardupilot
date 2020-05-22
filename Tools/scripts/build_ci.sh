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
export CHIBIOS_GIT_VERSION="ci_test"
export CCACHE_SLOPPINESS="include_file_ctime,include_file_mtime"
autotest_args=""

# If CI_BUILD_TARGET is not set, build 3 different ones
if [ -z "$CI_BUILD_TARGET" ]; then
    CI_BUILD_TARGET="sitl linux fmuv3"
fi

declare -A waf_supported_boards

waf=modules/waf/waf-light

# get list of boards supported by the waf build
for board in $($waf list_boards | head -n1); do waf_supported_boards[$board]=1; done

echo "Targets: $CI_BUILD_TARGET"
echo "Compiler: $c_compiler"

pymavlink_installed=0

function run_autotest() {
    NAME="$1"
    BVEHICLE="$2"
    RVEHICLE="$3"

    if [ $pymavlink_installed -eq 0 ]; then
        echo "Installing pymavlink"
        git submodule init
        git submodule update
        (cd modules/mavlink/pymavlink && python setup.py build install --user)
        pymavlink_installed=1
    fi
    unset BUILDROOT
    echo "Running SITL $NAME test"

    w=""
    if [ $c_compiler == "clang" ]; then
        w="$w --check-c-compiler=clang --check-cxx-compiler=clang++"
    fi
    if [ $NAME == "Rover" ]; then
        w="$w --enable-math-check-indexes"
    fi
    if [ "x$CI_BUILD_DEBUG" != "x" ]; then
        w="$w --debug"
    fi
    Tools/autotest/autotest.py --waf-configure-args="$w" "$BVEHICLE" "$RVEHICLE"
    ccache -s && ccache -z
}

for t in $CI_BUILD_TARGET; do
    # special case for SITL testing in CI
    if [ "$t" == "sitltest-copter" ]; then
        run_autotest "Copter" "build.ArduCopter" "fly.ArduCopter"
        continue
    fi
    if [ "$t" == "sitltest-plane" ]; then
        run_autotest "Plane" "build.ArduPlane" "fly.ArduPlane"
        continue
    fi
    if [ "$t" == "sitltest-quadplane" ]; then
        run_autotest "QuadPlane" "build.ArduPlane" "fly.QuadPlane"
        continue
    fi
    if [ "$t" == "sitltest-rover" ]; then
        run_autotest "Rover" "build.APMrover2" "drive.APMrover2"
        continue
    fi
    if [ "$t" == "sitltest-balancebot" ]; then
        run_autotest "BalanceBot" "build.APMrover2" "drive.BalanceBot"
        continue
    fi
    if [ "$t" == "sitltest-sub" ]; then
        run_autotest "Sub" "build.ArduSub" "dive.ArduSub"
        continue
    fi

    if [ "$t" == "unit-tests" ]; then
        run_autotest "Unit Tests" "build.unit_tests" "run.unit_tests"
        continue
    fi

    if [ "$t" == "revo-bootloader" ]; then
        echo "Building revo bootloader"
        $waf configure --board revo-mini --bootloader
        $waf clean
        $waf bootloader
        continue
    fi

    if [ "$t" == "periph-build" ]; then
        echo "Building f103 bootloader"
        $waf configure --board f103-GPS --bootloader
        $waf clean
        $waf bootloader
        echo "Building f103 peripheral fw"
        $waf configure --board f103-GPS
        $waf clean
        $waf AP_Periph
        continue
    fi
    
    if [ "$t" == "CubeOrange-bootloader" ]; then
        echo "Building CubeOrange bootloader"
        $waf configure --board CubeOrange --bootloader
        $waf clean
        $waf bootloader
        continue
    fi

    if [ "$t" == "stm32f7" ]; then
        echo "Building mRoX21-777/"
        $waf configure --board mRoX21-777
        $waf clean
        $waf plane
        continue
    fi

    if [ "$t" == "stm32h7" ]; then
        echo "Building Durandal"
        $waf configure --board Durandal
        $waf clean
        $waf copter
        continue
    fi

    if [ "$t" == "fmuv2-plane" ]; then
        echo "Building fmuv2 plane"
        $waf configure --board fmuv2
        $waf clean
        $waf plane
        continue
    fi
    
    if [ "$t" == "iofirmware" ]; then
        echo "Building iofirmware"
        $waf configure --board iomcu
        $waf clean
        $waf iofirmware
        continue
    fi

    if [ "$t" == "configure-all" ]; then
        echo "Checking configure of all boards"
        ./Tools/scripts/configure_all.py
        continue
    fi

    if [[ -n ${waf_supported_boards[$t]} && -z ${CI_CRON_JOB+1} ]]; then
        echo "Starting waf build for board ${t}..."
        $waf configure --board "$t" \
                --enable-benchmarks \
                --enable-header-checks \
                --check-c-compiler="$c_compiler" \
                --check-cxx-compiler="$cxx_compiler"
        $waf clean
        $waf copter
        $waf plane
        ccache -s && ccache -z

        if [[ $t == linux ]]; then
            $waf check
        fi
    fi
done

python Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
python Tools/autotest/param_metadata/param_parse.py --vehicle ArduPlane

echo build OK
exit 0

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

# If CI_BUILD_TARGET is not set, build 4 different ones
if [ -z "$CI_BUILD_TARGET" ]; then
    CI_BUILD_TARGET="sitl linux fmuv3 omnibusf4pro-one"
fi

waf=modules/waf/waf-light

echo "Targets: $CI_BUILD_TARGET"
echo "Compiler: $c_compiler"

pymavlink_installed=0
mavproxy_installed=0

function run_autotest() {
    NAME="$1"
    BVEHICLE="$2"
    RVEHICLE="$3"

    # report on what cpu's we have for later log review if needed
    cat /proc/cpuinfo

    if [ $mavproxy_installed -eq 0 ]; then
        echo "Installing MAVProxy"
        pushd /tmp
          git clone --recursive https://github.com/ardupilot/MAVProxy
          pushd MAVProxy
            python setup.py build install --user --force
          popd
        popd
        mavproxy_installed=1
        # now uninstall the version of pymavlink pulled in by MAVProxy deps:
        python -m pip uninstall -y pymavlink
    fi
    if [ $pymavlink_installed -eq 0 ]; then
        echo "Installing pymavlink"
        git submodule update --init --recursive
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
    if [ $NAME == "Examples" ]; then
        w="$w --speedup=5 --timeout=14400 --debug --no-clean"
    fi
    Tools/autotest/autotest.py --show-test-timings --waf-configure-args="$w" "$BVEHICLE" "$RVEHICLE"
    ccache -s && ccache -z
}

for t in $CI_BUILD_TARGET; do
    # special case for SITL testing in CI
    if [ "$t" == "sitltest-heli" ]; then
        run_autotest "Heli" "build.Helicopter" "test.Helicopter"
        continue
    fi
    # travis-ci
    if [ "$t" == "sitltest-copter-tests1" ]; then
        run_autotest "Copter" "build.Copter" "test.CopterTests1"
        continue
    fi
    #github actions ci
    if [ "$t" == "sitltest-copter-tests1a" ]; then
        run_autotest "Copter" "build.Copter" "test.CopterTests1a"
        continue
    fi
    if [ "$t" == "sitltest-copter-tests1b" ]; then
        run_autotest "Copter" "build.Copter" "test.CopterTests1b"
        continue
    fi
    if [ "$t" == "sitltest-copter-tests1c" ]; then
        run_autotest "Copter" "build.Copter" "test.CopterTests1c"
        continue
    fi
    if [ "$t" == "sitltest-copter-tests1d" ]; then
        run_autotest "Copter" "build.Copter" "test.CopterTests1d"
        continue
    fi
    if [ "$t" == "sitltest-copter-tests1e" ]; then
        run_autotest "Copter" "build.Copter" "test.CopterTests1e"
        continue
    fi

    # travis-ci
    if [ "$t" == "sitltest-copter-tests2" ]; then
        run_autotest "Copter" "build.Copter" "test.CopterTests2"
        continue
    fi
    #github actions ci
    if [ "$t" == "sitltest-copter-tests2a" ]; then
        run_autotest "Copter" "build.Copter" "test.CopterTests2a"
        continue
    fi
    if [ "$t" == "sitltest-copter-tests2b" ]; then
        run_autotest "Copter" "build.Copter" "test.CopterTests2b"
        continue
    fi
    if [ "$t" == "sitltest-can" ]; then
        echo "Building SITL Periph GPS"
        $waf configure --board sitl
        $waf copter
        run_autotest "Copter" "build.SITLPeriphGPS" "test.CAN"
        continue
    fi
    if [ "$t" == "sitltest-plane" ]; then
        run_autotest "Plane" "build.Plane" "test.Plane"
        continue
    fi
    if [ "$t" == "sitltest-quadplane" ]; then
        run_autotest "QuadPlane" "build.Plane" "test.QuadPlane"
        continue
    fi
    if [ "$t" == "sitltest-rover" ]; then
        run_autotest "Rover" "build.Rover" "test.Rover"
        continue
    fi
    if [ "$t" == "sitltest-tracker" ]; then
        run_autotest "Tracker" "build.Tracker" "test.Tracker"
        continue
    fi
    if [ "$t" == "sitltest-balancebot" ]; then
        run_autotest "BalanceBot" "build.Rover" "test.BalanceBot"
        continue
    fi
    if [ "$t" == "sitltest-sub" ]; then
        run_autotest "Sub" "build.Sub" "test.Sub"
        continue
    fi

    if [ "$t" == "unit-tests" ]; then
        run_autotest "Unit Tests" "build.unit_tests" "run.unit_tests"
        continue
    fi

    if [ "$t" == "examples" ]; then
        ./waf configure --board=linux --debug
        ./waf examples
        run_autotest "Examples" "--no-clean" "run.examples"
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
        echo "Building f303 bootloader"
        $waf configure --board f303-Universal --bootloader
        $waf clean
        $waf bootloader
        echo "Building f303 peripheral fw"
        $waf configure --board f303-Universal
        $waf clean
        $waf AP_Periph
        echo "Building CubeOrange peripheral fw"
        $waf configure --board CubeOrange-periph
        $waf clean
        $waf AP_Periph
        echo "Building G4-ESC peripheral fw"
        $waf configure --board G4-ESC
        $waf clean
        $waf AP_Periph
        echo "Building FreeflyRTK peripheral fw"
        $waf configure --board FreeflyRTK
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
        $waf configure --Werror --board mRoX21-777
        $waf clean
        $waf plane

        # test bi-directional dshot build
        echo "Building KakuteF7Mini"
        $waf configure --Werror --board KakuteF7Mini

        # test bi-directional dshot build and smallest flash
        echo "Building KakuteF7"
        $waf configure --Werror --board KakuteF7
        $waf clean
        $waf copter
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

    if [ "$t" == "navigator" ]; then
        echo "Building navigator"
        $waf configure --board navigator --toolchain=arm-linux-musleabihf
        $waf sub --static
        continue
    fi

    if [ "$t" == "replay" ]; then
        echo "Building replay"
        $waf configure --board sitl --debug --disable-scripting
        $waf replay
        echo "Building AP_DAL standalone test"
        $waf configure --board sitl --debug --disable-scripting --no-gcs
        $waf --target tools/AP_DAL_Standalone
        $waf clean
        continue
    fi

    if [ "$t" == "python-cleanliness" ]; then
        echo "Checking Python code cleanliness"
        ./Tools/scripts/run_flake8.py
        continue
    fi

    if [ "$t" == "configure-all" ]; then
        echo "Checking configure of all boards"
        ./Tools/scripts/configure_all.py
        continue
    fi

    if [[ -z ${CI_CRON_JOB+1} ]]; then
        echo "Starting waf build for board ${t}..."
        $waf configure --board "$t" \
                --enable-benchmarks \
                --enable-header-checks \
                --check-c-compiler="$c_compiler" \
                --check-cxx-compiler="$cxx_compiler"
        $waf clean
        $waf all
        ccache -s && ccache -z

        if [[ $t == "linux" ]]; then
            $waf check
        fi
        continue
    fi
done

python Tools/autotest/param_metadata/param_parse.py --vehicle Rover
python Tools/autotest/param_metadata/param_parse.py --vehicle AntennaTracker
python Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
python Tools/autotest/param_metadata/param_parse.py --vehicle ArduPlane
python Tools/autotest/param_metadata/param_parse.py --vehicle ArduSub

echo build OK
exit 0

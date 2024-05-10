#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

XOLDPWD=$PWD  # profile changes directory :-(

. ~/.profile

cd $XOLDPWD

set -ex

# CXX and CC are exported by default by travis
c_compiler=${CC:-gcc}
cxx_compiler=${CXX:-g++}

export BUILDROOT=/tmp/ci.build
rm -rf $BUILDROOT
export GIT_VERSION="abcdef"
export GIT_VERSION_INT="15"
export CHIBIOS_GIT_VERSION="12345667"
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

function install_pymavlink() {
    if [ $pymavlink_installed -eq 0 ]; then
        echo "Installing pymavlink"
        git submodule update --init --recursive --depth 1
        (cd modules/mavlink/pymavlink && python setup.py build install --user)
        pymavlink_installed=1
    fi
}

function install_mavproxy() {
    if [ $mavproxy_installed -eq 0 ]; then
        echo "Installing MAVProxy"
        pushd /tmp
          git clone https://github.com/ardupilot/MAVProxy --depth 1
          pushd MAVProxy
            python setup.py build install --user --force
          popd
        popd
        mavproxy_installed=1
        # now uninstall the version of pymavlink pulled in by MAVProxy deps:
        python -m pip uninstall -y pymavlink
    fi
}

function run_autotest() {
    NAME="$1"
    BVEHICLE="$2"
    RVEHICLE="$3"

    # report on what cpu's we have for later log review if needed
    cat /proc/cpuinfo

    install_mavproxy
    install_pymavlink
    unset BUILDROOT
    echo "Running SITL $NAME test"

    w=""
    if [ $c_compiler == "clang" ]; then
        w="$w --check-c-compiler=clang --check-cxx-compiler=clang++"
    fi
    if [ "$NAME" == "Rover" ]; then
        w="$w --enable-math-check-indexes"
    fi
    if [ "x$CI_BUILD_DEBUG" != "x" ]; then
        w="$w --debug"
    fi
    if [ "$NAME" == "Plane" ]; then
        w="$w --num-aux-imus=2"
    fi
    if [ "$NAME" == "Examples" ]; then
        w="$w --speedup=5 --timeout=14400 --debug --no-clean"
    fi
    Tools/autotest/autotest.py --show-test-timings --junit --waf-configure-args="$w" "$BVEHICLE" "$RVEHICLE"
    ccache -s && ccache -z
}

for t in $CI_BUILD_TARGET; do
    # special case for SITL testing in CI
    if [ "$t" == "sitltest-heli" ]; then
        run_autotest "Heli" "build.Helicopter" "test.Helicopter"
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
        run_autotest "Copter" "build.SITLPeriphUniversal" "test.CAN"
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
        sudo apt-get update || /bin/true
        sudo apt-get install -y ppp || /bin/true
        run_autotest "Rover" "build.Rover" "test.Rover"
        continue
    fi
    if [ "$t" == "sitltest-sailboat" ]; then
        run_autotest "Rover" "build.Rover" "test.Sailboat"
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
    if [ "$t" == "sitltest-blimp" ]; then
        run_autotest "Blimp" "build.Blimp" "test.Blimp"
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
        if [ -f ~/alternate_build/revo-mini/bin/AP_Bootloader.bin ]; then
            rm -r ~/alternate_build
        fi
        $waf configure --board revo-mini --bootloader --out ~/alternate_build
        $waf clean
        $waf bootloader
        # check if bootloader got built under alternate_build
        if [ ! -f ~/alternate_build/revo-mini/bin/AP_Bootloader.bin ]; then
            echo "alternate build output directory Test failed"
            exit 1
        fi
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
        echo "Building HerePro peripheral fw"
        $waf configure --board HerePro
        $waf clean
        $waf AP_Periph
        echo "Building CubeOrange-periph peripheral fw"
        $waf configure --board CubeOrange-periph
        $waf clean
        $waf AP_Periph
        echo "Building HerePro bootloader"
        $waf configure --board HerePro --bootloader
        $waf clean
        $waf bootloader
        echo "Building G4-ESC peripheral fw"
        $waf configure --board G4-ESC
        $waf clean
        $waf AP_Periph
        echo "Building Nucleo-L496 peripheral fw"
        $waf configure --board Nucleo-L496
        $waf clean
        $waf AP_Periph
        echo "Building Nucleo-L496 peripheral fw"
        $waf configure --board Nucleo-L476
        $waf clean
        $waf AP_Periph
        echo "Building Sierra-L431 peripheral fw"
        $waf configure --board Sierra-L431
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

    if [ "$t" == "CubeRedPrimary-bootloader" ]; then
        echo "Building CubeRedPrimary bootloader"
        $waf configure --board CubeRedPrimary --bootloader
        $waf clean
        $waf bootloader
        continue
    fi

    if [ "$t" == "fmuv3-bootloader" ]; then
        echo "Building fmuv3 bootloader"
        $waf configure --board fmuv3 --bootloader
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
        $waf clean
        $waf copter

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
        echo "Building CPUInfo"
        $waf --target=tool/CPUInfo

        # test external flash build
        echo "Building SPRacingH7"
        $waf configure --Werror --board SPRacingH7
        $waf clean
        $waf copter
        continue
    fi

    if [ "$t" == "stm32h7-debug" ]; then
        echo "Building Durandal"
        $waf configure --board Durandal --debug
        $waf clean
        $waf copter
        continue
    fi

    if [ "$t" == "CubeOrange-ODID" ]; then
        echo "Building CubeOrange-ODID"
        $waf configure --board CubeOrange-ODID
        $waf clean
        $waf copter
        $waf plane
        continue
    fi

    if [ "$t" == "CubeOrange-PPP" ]; then
        echo "Building CubeOrange-PPP"
        $waf configure --board CubeOrange --enable-ppp
        $waf clean
        $waf copter
        continue
    fi

    if [ "$t" == "CubeOrange-SOHW" ]; then
        echo "Building CubeOrange-SOHW"
        Tools/scripts/sitl-on-hardware/sitl-on-hw.py --vehicle plane --simclass Plane --board CubeOrange
        continue
    fi

    if [ "$t" == "Pixhawk6X-PPPGW" ]; then
        echo "Building Pixhawk6X-PPPGW"
        $waf configure --board Pixhawk6X-PPPGW
        $waf clean
        $waf AP_Periph
        $waf configure --board Pixhawk6X-PPPGW --bootloader
        $waf clean
        $waf bootloader
        continue
    fi
    
    if [ "$t" == "dds-stm32h7" ]; then
        echo "Building with DDS support on a STM32H7"
        $waf configure --board Durandal --enable-dds
        $waf clean
        $waf copter
        $waf plane
        continue
    fi

    if [ "$t" == "dds-sitl" ]; then
        echo "Building with DDS support on SITL"
        $waf configure --board sitl --enable-dds
        $waf clean
        $waf copter
        $waf plane
        $waf tests
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
        Tools/scripts/build_iofirmware.py
        # now clean up the stuff that's copied into the source tree:
        git checkout Tools/IO_Firmware/
        continue
    fi

    if [ "$t" == "navigator" ]; then
        echo "Building navigator"
        $waf configure --board navigator --toolchain=arm-linux-musleabihf
        $waf sub --static
        ./Tools/scripts/firmware_version_decoder.py -f build/navigator/bin/ardusub --expected-hash $GIT_VERSION
        continue
    fi

    if [ "$t" == "replay" ]; then
        echo "Building replay"
        $waf configure --board sitl --debug --disable-scripting

        $waf replay
        echo "Building AP_DAL standalone test"
        $waf configure --board sitl --debug --disable-scripting --no-gcs

        $waf --target tool/AP_DAL_Standalone
        $waf clean
        continue
    fi

    if [ "$t" == "validate_board_list" ]; then
        echo "Validating board list"
        ./Tools/autotest/validate_board_list.py
        continue
    fi

    if [ "$t" == "check_autotest_options" ]; then
        echo "Checking autotest options"
        install_mavproxy
        install_pymavlink
        ./Tools/autotest/autotest.py --help
        ./Tools/autotest/autotest.py --list
        ./Tools/autotest/autotest.py --list-subtests
        continue
    fi

    if [ "$t" == "signing" ]; then
        echo "Building signed firmwares"
        sudo apt-get update
        sudo apt-get install -y python3-dev
        python3 -m pip install pymonocypher==3.1.3.2
        ./Tools/scripts/signing/generate_keys.py testkey
        $waf configure --board CubeOrange-ODID --signed-fw --private-key testkey_private_key.dat
        $waf copter
        $waf configure --board MatekL431-DShot --signed-fw --private-key testkey_private_key.dat
        $waf AP_Periph
        ./Tools/scripts/build_bootloaders.py --signing-key testkey_public_key.dat CubeOrange-ODID
        ./Tools/scripts/build_bootloaders.py --signing-key testkey_public_key.dat MatekL431-DShot
        continue
    fi

    if [ "$t" == "python-cleanliness" ]; then
        echo "Checking Python code cleanliness"
        ./Tools/scripts/run_flake8.py
        continue
    fi

    if [ "$t" == "astyle-cleanliness" ]; then
        echo "Checking AStyle code cleanliness"
        ./Tools/scripts/run_astyle.py --dry-run
        continue
    fi

    if [ "$t" == "configure-all" ]; then
        echo "Checking configure of all boards"
        ./Tools/scripts/configure_all.py
        continue
    fi

    if [ "$t" == "build-options-defaults-test" ]; then
        install_pymavlink
        echo "Checking default options in build_options.py work"
        time ./Tools/autotest/test_build_options.py \
             --no-disable-all \
             --no-disable-none \
             --no-disable-in-turn \
             --no-enable-in-turn \
             --board=CubeOrange \
             --build-targets=copter \
             --build-targets=plane
        echo "Checking all/none options in build_options.py work"
        time ./Tools/autotest/test_build_options.py \
             --no-disable-in-turn \
             --no-enable-in-turn \
             --build-targets=copter \
             --build-targets=plane
        continue
    fi

    if [ "$t" == "param_parse" ]; then
        for v in Rover AntennaTracker ArduCopter ArduPlane ArduSub Blimp AP_Periph; do
            python Tools/autotest/param_metadata/param_parse.py --vehicle $v
        done
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

echo build OK
exit 0

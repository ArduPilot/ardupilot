#!/bin/sh

set -e
#set -x

# EXECUTE enough code via the autotest tool to see coverage results
# afterwards, but don't build/rebuild anything our aim here is to try
# to execute as many code path/s as we have available to us, and we'll
# afterward report on the percentage of code executed and not executed
# etc.

REPORT_DIR="reports/lcov-report"

INFO_FILE="$REPORT_DIR/lcov.info"
INFO_FILE_BASE="$REPORT_DIR/lcov_base.info"

LCOV_LOG="GCOV_lcov.log"
GENHTML_LOG="GCOV_genhtml.log"

sep="##############################################"

initialize_coverage() {
    echo $sep
    echo "Initilizing Coverage"
    echo $sep
    echo "Removing previous build"
    rm -rf "$REPORT_DIR"
    mkdir -p "$REPORT_DIR"
    echo "Zeroing previous build"
    lcov --zerocounters --directory $PWD
    echo "Initilizing Coverage with current build"
    lcov --no-external --initial --capture --exclude "$PWD/modules/uavcan/*" --exclude "$PWD/build/sitl/modules/*" --directory $PWD -o "$INFO_FILE_BASE" 2>&1 | tee $LCOV_LOG
}

run_full_coverage() {
    export CCFLAGS="$CCFLAGS -fprofile-arcs -ftest-coverage"
    export CXXFLAGS="$CXXFLAGS -fprofile-arcs -ftest-coverage"
    export LINKFLAGS="$LINKFLAGS -lgcov -coverage"
    export COVERAGE=True

    SPEEDUP=5
    TIMEOUT=14400

    OPTS="--timeout=$TIMEOUT --debug --no-clean"

    echo $sep
    echo "Running full test suite"
    echo $sep
    echo "Removing previous build binaries"
    rm -rf build
    # Run examples
    echo $sep
    echo "Building examples and SITL binaries"
    ./waf configure --board=linux --debug
    ./waf examples
    ./waf configure --debug
    ./waf
    initialize_coverage
    echo $sep
    echo "Running tests"
    ./Tools/autotest/autotest.py $OPTS "--speedup=$SPEEDUP" run.examples

    # Run unit tests
    ./Tools/autotest/autotest.py $OPTS build.unit_tests run.unit_tests

    # Run main vehicle tests
    ./Tools/autotest/autotest.py $OPTS build.Plane test.Plane test.QuadPlane
    ./Tools/autotest/autotest.py $OPTS build.Sub test.Sub
    ./Tools/autotest/autotest.py $OPTS build.Copter test.Copter
    ./Tools/autotest/autotest.py $OPTS build.Helicopter test.Helicopter
    ./Tools/autotest/autotest.py $OPTS build.Tracker test.Tracker
    ./Tools/autotest/autotest.py $OPTS build.Rover test.Rover
    #TODO add any other execution path/s we can to maximise the actually
    # used code, can we run other tests or things?  Replay, perhaps?
    update_coverage_stats
}

update_coverage_stats() {
    echo $sep
    echo "Generating Coverage statistics"
    echo $sep
    lcov --no-external --capture --directory $PWD -o "$INFO_FILE" 2>&1 | tee $LCOV_LOG
    lcov --add-tracefile "$INFO_FILE_BASE" --add-tracefile "$INFO_FILE"
    # remove files we do not intentionally test:
    lcov --remove "$INFO_FILE" ".waf*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG
    lcov --remove "$INFO_FILE" "$PWD/modules/gtest/*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG
    lcov --remove "$INFO_FILE" "$PWD/modules/uavcan/*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG
    lcov --remove "$INFO_FILE" "$PWD/build/linux/libraries/*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG
    lcov --remove "$INFO_FILE" "$PWD/build/sitl/libraries/*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG
    lcov --remove "$INFO_FILE" "$PWD/build/sitl/modules/*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG

    genhtml "$INFO_FILE" -o "$REPORT_DIR" 2>&1 | tee $GENHTML_LOG

    echo "Coverage successful. Open $REPORT_DIR/index.html"
}


usage () {
        echo "Usage: $(basename $0) [-iru]" 2>&1
        echo '   -i   Initialise ArduPilot for coverage. It should be run after building the binaries.'
        echo '   -f   Run ArduPilot full coverage. This will run all tests and example. It is really long.'
        echo '   -u   Update coverage statistics. To be used after running some tests.'
        exit 1
}

if [ ${#} -eq 0 ]; then
   usage
fi

if [ ${#} -gt 1 ]; then
   echo " Only one option is allowed"
   exit 1
fi

# Define list of arguments expected in the input
optstring=":ifu"

while getopts ${optstring} arg; do
  case "${arg}" in
    i) initialize_coverage ;;
    f) run_full_coverage ;;
    u) update_coverage_stats ;;

    ?)
      echo "Invalid option: -${OPTARG}."
      echo
      usage
      ;;
  esac
done

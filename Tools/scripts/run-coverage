#!/bin/sh

set -e
set -x

# EXECUTE enough code via the autotest tool to see coverage results
# afterwards, but don't build/rebuild anything our aim here is to try
# to execute as many code path/s as we have available to us, and we'll
# afterward report on the percentage of code executed and not executed
# etc.

export CCFLAGS="$CCFLAGS -fprofile-arcs -ftest-coverage"
export CXXFLAGS="$CXXFLAGS -fprofile-arcs -ftest-coverage"
export LINKFLAGS="$LINKFLAGS -lgcov -coverage"

SPEEDUP=5
TIMEOUT=14400

OPTS="--speedup=$SPEEDUP --timeout=$TIMEOUT --debug --no-clean"

rm -rf build

# Run examples
./waf configure --board=linux --debug
./waf examples
./Tools/autotest/autotest.py $OPTS run.examples

# Run unit tests
./Tools/autotest/autotest.py $OPTS build.unit_tests run.unit_tests

# Run main vehicle tests
./Tools/autotest/autotest.py $OPTS build.ArduPlane fly.ArduPlane fly.QuadPlane
./Tools/autotest/autotest.py $OPTS build.ArduSub dive.ArduSub
./Tools/autotest/autotest.py $OPTS build.ArduCopter fly.ArduCopter
./Tools/autotest/autotest.py $OPTS build.Helicopter fly.CopterAVC
./Tools/autotest/autotest.py $OPTS build.AntennaTracker test.AntennaTracker
./Tools/autotest/autotest.py $OPTS build.APMrover2 drive.APMrover2

#TODO add any other execution path/s we can to maximise the actually
# used code, can we run other tests or things?  Replay, perhaps?

REPORT_DIR="reports/lcov-report"
rm -rf "$REPORT_DIR"
mkdir -p "$REPORT_DIR"

INFO_FILE="$REPORT_DIR/lcov.info"

LCOV_LOG="GCOV_lcov.log"
GENHTML_LOG="GCOV_genhtml.log"

lcov --no-external --capture --directory $PWD -o "$INFO_FILE" 2>&1 | tee $LCOV_LOG
# remove files we do not intentionally test:
lcov --remove "$INFO_FILE" ".waf*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG
lcov --remove "$INFO_FILE" "$PWD/modules/gtest/*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG
lcov --remove "$INFO_FILE" "$PWD/build/linux/libraries/*" -o "$INFO_FILE" 2>&1 | tee -a $LCOV_LOG

genhtml "$INFO_FILE" -o "$REPORT_DIR" 2>&1 | tee $GENHTML_LOG

echo "Coverage successful. Open $REPORT_DIR/index.html"

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
./Tools/autotest/autotest.py $OPTS build.Plane test.Plane test.QuadPlane
./Tools/autotest/autotest.py $OPTS build.Sub test.Sub
./Tools/autotest/autotest.py $OPTS build.Copter test.Copter
./Tools/autotest/autotest.py $OPTS build.Helicopter test.Helicopter
./Tools/autotest/autotest.py $OPTS build.Tracker test.Tracker
./Tools/autotest/autotest.py $OPTS build.Rover test.Rover

#TODO add any other execution path/s we can to maximise the actually
# used code, can we run other tests or things?  Replay, perhaps?

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

$SCRIPTPATH/update-coverage.sh


#!/bin/bash

set -e
set -x

PARAMS_DIR="../buildlogs/Parameters"

# work from either APM directory or above
[ -d ArduPlane ] || cd APM

/bin/mkdir -p "$PARAMS_DIR"

generate_parameters() {
    VEHICLE="$1"

    # generate Parameters.html, Parameters.rst etc etc:
    ./Tools/autotest/param_metadata/param_parse.py --vehicle $VEHICLE

    # stash some of the results away:
    VEHICLE_PARAMS_DIR="$PARAMS_DIR/$VEHICLE"
    mkdir -p "$VEHICLE_PARAMS_DIR"
    /bin/cp Parameters.wiki Parameters.html *.pdef.xml "$VEHICLE_PARAMS_DIR/"
    if [ -e "Parameters.rst" ]; then
	/bin/cp Parameters.rst "$VEHICLE_PARAMS_DIR/"
    fi
}


generate_parameters ArduPlane

generate_parameters ArduCopter

generate_parameters Rover

generate_parameters ArduSub

generate_parameters AntennaTracker

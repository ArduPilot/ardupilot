#!/bin/bash

set -e
set -x

DIR="../buildlogs/LogMessages"

# work from either APM directory or above
[ -d ArduPlane ] || cd APM

/bin/mkdir -p "$DIR"

generate_log_message_documentation() {
    VEHICLE="$1"

    # generate Parameters.html, Parameters.rst etc etc:
    ./Tools/autotest/logger_metadata/parse.py --vehicle "$VEHICLE"

    # stash some of the results away:
    VEHICLE_DIR="$DIR/$VEHICLE"
    mkdir -p "$VEHICLE_DIR"
    /bin/cp LogMessages.* "$VEHICLE_DIR/"
}

for vehicle in Rover Plane Copter Tracker; do
    generate_log_message_documentation "$vehicle"
done

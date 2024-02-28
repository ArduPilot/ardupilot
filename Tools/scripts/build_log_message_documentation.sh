#!/usr/bin/env bash

set -e
set -x

if [ "x$BUILDLOGS" = "x" ]; then
    BUILDLOGS="../buildlogs"
fi
DIR="$BUILDLOGS/LogMessages"

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
    gzip -9 <"$VEHICLE_DIR"/LogMessages.xml >"$VEHICLE_DIR"/LogMessages.xml.gz.new && mv "$VEHICLE_DIR"/LogMessages.xml.gz.new "$VEHICLE_DIR"/LogMessages.xml.gz
    xz -e <"$VEHICLE_DIR"/LogMessages.xml >"$VEHICLE_DIR"/LogMessages.xml.xz.new && mv "$VEHICLE_DIR"/LogMessages.xml.xz.new "$VEHICLE_DIR"/LogMessages.xml.xz
}

for vehicle in Rover Plane Copter Tracker Blimp Sub; do
    generate_log_message_documentation "$vehicle"
done

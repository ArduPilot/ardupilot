#!/usr/bin/env bash

set -e
set -x

if [ "x$BUILDLOGS" = "x" ]; then
    BUILDLOGS="../buildlogs"
fi
PARAMS_DIR="$BUILDLOGS/Parameters"

# work from either APM directory or above
[ -d ArduPlane ] || cd APM

/bin/mkdir -p "$PARAMS_DIR"

GIT_SHA=$(git rev-parse HEAD)
GIT_TAG=$(git describe --tags --exact-match 2>/dev/null || true)

generate_parameters() {
    VEHICLE="$1"

    # generate Parameters.html, Parameters.rst etc etc:
    GIT_TAG_ARG=""
    [ -n "$GIT_TAG" ] && GIT_TAG_ARG="--git-tag $GIT_TAG"
    # shellcheck disable=SC2086
    ./Tools/autotest/param_metadata/param_parse.py --vehicle $VEHICLE --git-sha "$GIT_SHA" $GIT_TAG_ARG --compress

    # stash some of the results away:
    VEHICLE_PARAMS_DIR="$PARAMS_DIR/$VEHICLE"
    mkdir -p "$VEHICLE_PARAMS_DIR"
    /bin/cp Parameters.html *.pdef.xml *.pdef.xml.xz "$VEHICLE_PARAMS_DIR/"
    gzip -9 <"$VEHICLE_PARAMS_DIR"/apm.pdef.xml >"$VEHICLE_PARAMS_DIR"/apm.pdef.xml.gz.new && mv "$VEHICLE_PARAMS_DIR"/apm.pdef.xml.gz.new "$VEHICLE_PARAMS_DIR"/apm.pdef.xml.gz
    if [ -e "Parameters.rst" ]; then
	/bin/cp Parameters.rst "$VEHICLE_PARAMS_DIR/"
    fi
    if [ -e "ParametersLatex.rst" ]; then
    /bin/cp ParametersLatex.rst "$VEHICLE_PARAMS_DIR/"
    fi
    F="apm.pdef.json"
    if [ -e "$F" ]; then
	    /bin/cp "$F" "$F.xz" "$VEHICLE_PARAMS_DIR/"
    fi
}

generate_parameters ArduPlane

generate_parameters ArduCopter

generate_parameters Rover

generate_parameters ArduSub

generate_parameters AntennaTracker

generate_parameters AP_Periph

generate_parameters Blimp

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

generate_parameters() {
    VEHICLE="$1"

    TAG_PREFIX="$VEHICLE"
    case "$VEHICLE" in
        ArduCopter) TAG_PREFIX="Copter" ;;
        ArduPlane) TAG_PREFIX="Plane" ;;
        ArduSub) TAG_PREFIX="Sub" ;;
        AntennaTracker) TAG_PREFIX="Tracker" ;;
    esac
    GIT_TAG=$(git describe --tags --exact-match --match "$TAG_PREFIX-[0-9]*.[0-9]*.[0-9]*" 2>/dev/null || true)
    if [ -z "$GIT_TAG" ] && [ "$VEHICLE" = "ArduSub" ]; then
        GIT_TAG=$(git describe --tags --exact-match --match "ArduSub-[0-9]*.[0-9]*.[0-9]*" 2>/dev/null || true)
    fi

    # Generate Parameters.html, Parameters.rst etc etc.  The flat files are
    # release metadata, so do not include the changing commit SHA.
    GIT_TAG_ARG=""
    [ -n "$GIT_TAG" ] && GIT_TAG_ARG="--git-tag $GIT_TAG"
    # shellcheck disable=SC2086
    ./Tools/autotest/param_metadata/param_parse.py --vehicle $VEHICLE $GIT_TAG_ARG

    # stash some of the results away:
    VEHICLE_PARAMS_DIR="$PARAMS_DIR/$VEHICLE"
    mkdir -p "$VEHICLE_PARAMS_DIR"
    /bin/cp Parameters.html *.pdef.xml "$VEHICLE_PARAMS_DIR/"
    gzip -9 <"$VEHICLE_PARAMS_DIR"/apm.pdef.xml >"$VEHICLE_PARAMS_DIR"/apm.pdef.xml.gz.new && mv "$VEHICLE_PARAMS_DIR"/apm.pdef.xml.gz.new "$VEHICLE_PARAMS_DIR"/apm.pdef.xml.gz
    xz -e <"$VEHICLE_PARAMS_DIR"/apm.pdef.xml >"$VEHICLE_PARAMS_DIR"/apm.pdef.xml.xz.new && mv "$VEHICLE_PARAMS_DIR"/apm.pdef.xml.xz.new "$VEHICLE_PARAMS_DIR"/apm.pdef.xml.xz
    if [ -e "Parameters.rst" ]; then
	/bin/cp Parameters.rst "$VEHICLE_PARAMS_DIR/"
    fi
    if [ -e "ParametersLatex.rst" ]; then
    /bin/cp ParametersLatex.rst "$VEHICLE_PARAMS_DIR/"
    fi
    F="apm.pdef.json"
    if [ -e "$F" ]; then
	    /bin/cp "$F" "$VEHICLE_PARAMS_DIR/"
        xz -e <"$VEHICLE_PARAMS_DIR"/"$F" >"$VEHICLE_PARAMS_DIR"/"$F.xz.new" && mv "$VEHICLE_PARAMS_DIR"/"$F.xz.new" "$VEHICLE_PARAMS_DIR"/"$F.xz"
    fi
}

generate_parameters ArduPlane

generate_parameters ArduCopter

generate_parameters Rover

generate_parameters ArduSub

generate_parameters AntennaTracker

generate_parameters AP_Periph

generate_parameters Blimp

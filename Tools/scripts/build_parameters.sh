#!/bin/bash

set -e
# set -x

WP_Auth_Dir="$HOME/WP_Auth"
PARAMS_DIR="../buildlogs/Parameters"

# work from either APM directory or above
[ -d ArduPlane ] || cd APM

# generate combined parameters files for all vehicles:
./Tools/autotest/param_metadata/param_parse.py > param.out || {
    echo "Parameter parsing failed"
    exit 1
}
# copy combined parameters files away:
/bin/mkdir -p "$PARAMS_DIR"
/bin/cp Parameters.wiki Parameters.html *.pdef.xml "$PARAMS_DIR"

generate_parameters() {
    VEHICLE="$1"
    URL="$2"
    AUTHFILE="$3"
    POST_TITLE="$4"

    # generate Parameters.html, Parameters.rst etc etc:
    ./Tools/autotest/param_metadata/param_parse.py --vehicle $VEHICLE

    # (Possibly) upload to the Wiki:
    if [ -d "$WP_Auth_Dir" ]; then
	if [ "$URL" != "NONE" ]; then
	    AUTHFILEPATH="$WP_Auth_Dir/$AUTHFILE"
	    ./Tools/scripts/update_wiki.py --url "$URL" $(cat $AUTHFILEPATH) --post-title="$POST_TITLE" Parameters.html
	fi
    fi

    # stash some of the results away:
    VEHICLE_PARAMS_DIR="$PARAMS_DIR/$VEHICLE"
    mkdir -p "$VEHICLE_PARAMS_DIR"
    /bin/cp Parameters.wiki Parameters.html *.pdef.xml "$VEHICLE_PARAMS_DIR/"
    if [ -e "Parameters.rst" ]; then
	/bin/cp Parameters.rst "$VEHICLE_PARAMS_DIR/"
    fi
}


generate_parameters ArduPlane http://plane.ardupilot.org plane.auth 'Plane Parameters'

generate_parameters ArduCopter http://copter.ardupilot.org copter.auth 'Copter Parameters'

generate_parameters APMrover2 http://rover.ardupilot.org rover.auth 'Rover Parameters'

generate_parameters ArduSub http://sub.ardupilot.org sub.auth 'Sub Parameters'

generate_parameters AntennaTracker NONE NONE 'AntennaTracker Parameters'

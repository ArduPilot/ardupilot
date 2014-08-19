#!/bin/bash

set -e

# work from either APM directory or above
[ -d ArduPlane ] || cd APM

./Tools/autotest/param_metadata/param_parse.py > param.out || {
    echo "Parameter parsing failed"
    exit 1
}
/bin/mkdir -p ../buildlogs/Parameters
/bin/cp Parameters.wiki Parameters.html *.pdef.xml ../buildlogs/Parameters/

[ -d $HOME/WP_Auth ] && {
    # now upload to WordPress
    ./Tools/autotest/param_metadata/param_parse.py --vehicle ArduPlane
    ./Tools/scripts/update_wiki.py --url http://plane.ardupilot.com $(cat $HOME/WP_Auth/plane.auth) --post-title='ArduPlane Parameters' Parameters.html
    mkdir -p ../buildlogs/Parameters/ArduPlane
    /bin/cp Parameters.wiki Parameters.html *.pdef.xml ../buildlogs/Parameters/ArduPlane

    ./Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
    ./Tools/scripts/update_wiki.py --url http://copter.ardupilot.com $(cat $HOME/WP_Auth/copter.auth) --post-title='ArduCopter Parameters' Parameters.html
    mkdir -p ../buildlogs/Parameters/ArduCopter
    /bin/cp Parameters.wiki Parameters.html *.pdef.xml ../buildlogs/Parameters/ArduCopter

    ./Tools/autotest/param_metadata/param_parse.py --vehicle APMrover2
    ./Tools/scripts/update_wiki.py --url http://rover.ardupilot.com $(cat $HOME/WP_Auth/rover.auth) --post-title='APMrover2 Parameters' Parameters.html
    mkdir -p ../buildlogs/Parameters/APMrover2
    /bin/cp Parameters.wiki Parameters.html *.pdef.xml ../buildlogs/Parameters/APMrover2

    ./Tools/autotest/param_metadata/param_parse.py --vehicle AntennaTracker
    #./Tools/scripts/update_wiki.py --url http://rover.ardupilot.com $(cat $HOME/WP_Auth/rover.auth) --post-title='APMrover2 Parameters' Parameters.html
    mkdir -p ../buildlogs/Parameters/AntennaTracker
    /bin/cp Parameters.wiki Parameters.html *.pdef.xml ../buildlogs/Parameters/AntennaTracker
}

#!/bin/bash

set -e

cd APM
./Tools/autotest/param_metadata/param_parse.py > param.out || exit 1
(cd ../APM.wiki && git pull --rebase)
cmp Parameters.wiki ../APM.wiki/APM_Parameters.wiki || {
    cp Parameters.wiki ../APM.wiki/APM_Parameters.wiki
    pushd ../APM.wiki
    git commit -m 'autotest updated parameters page' --author='autotest <autotest@tridgell.net>' APM_Parameters.wiki
    git push
    popd
}
/bin/mkdir -p ../buildlogs/Parameters
/bin/cp Parameters.wiki *.pdef.xml ../buildlogs/Parameters/

[ -d $HOME/WP_Auth ] && {
    # now upload to WordPress
    ./Tools/autotest/param_metadata/param_parse.py --vehicle ArduPlane
    ./Tools/scripts/update_wiki.py --url http://plane.ardupilot.com $(cat $HOME/WP_Auth/copter.auth) --post-title='ArduPlane Parameters' Parameters.html

    ./Tools/autotest/param_metadata/param_parse.py --vehicle ArduCopter
    ./Tools/scripts/update_wiki.py --url http://copter.ardupilot.com $(cat $HOME/WP_Auth/copter.auth) --post-title='ArduCopter Parameters' Parameters.html

    ./Tools/autotest/param_metadata/param_parse.py --vehicle APMrover2
    ./Tools/scripts/update_wiki.py --url http://rover.ardupilot.com $(cat $HOME/WP_Auth/rover.auth) --post-title='APMrover2 Parameters' Parameters.html
}

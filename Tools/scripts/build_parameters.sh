#!/bin/bash

set -e

cd APM
./Tools/autotest/param_metadata/param_parse.py > param.out
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

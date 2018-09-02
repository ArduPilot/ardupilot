#!/bin/bash

set -e
set -x
echo "---------- $0 start ----------"

# don't waste time rebuilding jsbsim if we already have a working copy of it
# this can save ~10 or minutes on some hardware
JSBBINARY=~/jsbsim/src/JSBSim
if [ -f $JSBBINARY ]; then
    echo "$JSBBINARY exists, skipping build. (to force rebuild then remove this file and re-run) "
else
    echo "$JSBBINARY does not exist, building it in your home folder:"
    cd ~ 
    rm -rf jsbsim
    git clone https://github.com/tridge/jsbsim.git
    cd jsbsim
    ./autogen.sh
    make -j2
fi
echo "---------- $0 end ----------"

#!/bin/bash

set -e
set -x
echo "---------- $0 start ----------"

# don't waste time rebuilding jsbsim if we already have a working copy of it
# this can save ~10 or minutes on some hardware
JSBBINARY=~/jsbsim/build/src/JSBSim
if [ -f $JSBBINARY ]; then
    echo "$JSBBINARY exists, skipping build. (to force rebuild then remove this file and re-run) "
else
    echo "$JSBBINARY does not exist, building it in your home folder:"
    cd ~ 
    rm -rf jsbsim
    git clone git://github.com/JSBSim-Team/jsbsim.git
    cd jsbsim
    mkdir build
    cd build
    cmake -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -mtune=native" -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native -mtune=native" -DCMAKE_BUILD_TYPE=Release ..
    make -j2
fi

if [[ ! -n $(echo $PATH | grep jsbsim) ]]; then
    echo "Add the JSBSim executable to your PATH using - export PATH=\$PATH:~/jsbsim/build/src"
    echo "Add the above command to ~/.bashrc to automatically set the path everytime a new terminal is launched"
fi

echo "---------- $0 end ----------"

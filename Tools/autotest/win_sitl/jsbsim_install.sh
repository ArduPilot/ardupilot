#!/bin/bash

#A simple script to install the APM SITL environment into cygwin

python -m ensurepip --user
python -m pip install --user future
git clone git://github.com/tridge/jsbsim.git
cd jsbsim
./autogen.sh
make
cp src/JSBSim.exe /usr/local/bin

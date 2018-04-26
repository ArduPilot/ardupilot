#!/bin/bash

#A simple script to install the APM SITL environment into cygwin

git clone git://github.com/ArduPilot/ardupilot.git
git clone git://github.com/tridge/jsbsim.git
cd jsbsim
./autogen.sh
make
cp src/JSBSim.exe /usr/local/bin
cd ../ardupilot
git submodule update --init --recursive
./modules/waf/waf-light configure --board=sitl

read -p "Add $HOME/ardupilot/Tools/autotest to your PATH [Y/n]?"
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
else
    echo "Skipping adding $HOME/ardupilot/Tools/autotest to PATH."
fi

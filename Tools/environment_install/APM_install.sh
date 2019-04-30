#!/bin/bash

#A simple script to install the APM SITL environment into cygwin

git clone git://github.com/ArduPilot/ardupilot.git
cd ./ardupilot
git submodule update --init --recursive
./modules/waf/waf-light configure --board=sitl

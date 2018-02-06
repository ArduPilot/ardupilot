#!/bin/bash

set -e
set -x

rm -rf jsbsim
git clone https://github.com/tridge/jsbsim.git
cd jsbsim
./autogen.sh
make -j2

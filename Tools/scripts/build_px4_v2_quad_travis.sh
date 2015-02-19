#!/bin/bash

. config.mk

set -e
set -x

pushd ../PX4Firmware
set +x
git remote add solo https://$1@github.com/3drobotics/PX4Firmware-solo.git
set -x
git fetch solo
git checkout solo/master
popd

pushd ArduCopter
make configure
make clean
make px4-cleandep
make px4-v2
popd

exit 0

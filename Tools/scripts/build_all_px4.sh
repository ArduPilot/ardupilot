#!/bin/bash
# build all targets for PX4
# This helps when doing large merges
# Andrew Tridgell, February 2013

set -e
set -x

. config.mk

echo "Testing ArduPlane build"
pushd ArduPlane
make px4-clean
make px4
make px4-io
popd

echo "Testing ArduCopter build"
pushd ArduCopter
make px4-clean
make px4
popd

echo "Testing APMrover2 build"
pushd APMrover2
make px4-clean
make px4
popd

exit 0

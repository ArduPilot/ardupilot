#!/bin/bash
# build all targets for PX4
# This helps when doing large merges
# Andrew Tridgell, February 2013

. config.mk

set -e
set -x

git submodule init
git submodule update --recursive

builds="px4-v1 px4-v2 px4-v3 px4-v4 px4-v4pro"

for b in $builds; do
    echo "Testing $b build"
    ./waf clean
    ./waf configure --board $b
    ./waf
done

exit 0

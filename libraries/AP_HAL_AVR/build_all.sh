#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

set -e
set -x

export PATH=/usr/lib/ccache:$PATH

TESTS=$(find libraries -name '*.pde' | xargs grep -l include..AP_HAL.h | xargs -i dirname '{}')

for b in $TESTS; do
    echo "TESTING $b"
    pushd $b
    make clean
    make
    popd
done

echo "All tests built OK"
exit 0

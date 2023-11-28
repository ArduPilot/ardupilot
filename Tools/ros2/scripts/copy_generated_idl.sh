#!/bin/bash

# This script copies IDL generated files from the build directory back to the source tree.
# These generated files are used in the build for devs who don't have microxrceddsgen.

set euf -o pipefail

rm -r build
./waf configure --board sitl
./waf plane

GEN_SRC=./build/sitl/libraries/AP_DDS/generated
GEN_DST=./libraries/AP_DDS
rm -rf $GEN_DST/generated

rsync -marv --include='*.h' --include='*.c' --include='*/' --exclude='*' ${GEN_SRC} ${GEN_DST}

echo "Add libraries/AP_DDS to git staging."

#!/bin/bash

# script to build cygwin binaries for using in MissionPlanner
# the contents of artifacts directory is uploaded to:
# https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/

# the script assumes you start in the root of the ardupilot git tree

set -x

rm -rf artifacts
mkdir artifacts

(
    python ./waf --color yes --toolchain i686-pc-cygwin --board sitl configure 2>&1
    python ./waf plane rover copter sub heli 2>&1
) | tee artifacts/build.txt

i686-pc-cygwin-g++ -print-sysroot

cp -v build/sitl/bin/arduplane artifacts/ArduPlane.elf
cp -v build/sitl/bin/arducopter artifacts/ArduCopter.elf
cp -v build/sitl/bin/arducopter-heli artifacts/ArduHeli.elf
cp -v build/sitl/bin/ardurover artifacts/ArduRover.elf
cp -v build/sitl/bin/ardusub artifacts/ArduSub.elf

cp -v /usr/i686-pc-cygwin/sys-root/usr/bin/*.dll artifacts/

git log -1 > artifacts/git.txt

#!/bin/bash

# script to build cygwin binaries for using in MissionPlanner
# the contents of artifacts directory is uploaded to:
# https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/

# the script assumes you start in the root of the ardupilot git tree

set -x

# TOOLCHAIN=i686-pc-cygwin
TOOLCHAIN=x86_64-pc-cygwin
GPP_COMPILER="${TOOLCHAIN}-g++"

$GPP_COMPILER -print-sysroot

SYS_ROOT=$($GPP_COMPILER -print-sysroot)
echo "SYS_ROOT=$SYS_ROOT"

git config --global --add safe.directory /cygdrive/d/a/ardupilot/ardupilot

rm -rf artifacts
mkdir artifacts

(
    python ./waf --color yes --toolchain $TOOLCHAIN --board sitl configure 2>&1
    python ./waf plane 2>&1
    python ./waf copter 2>&1
    python ./waf heli 2>&1
    python ./waf rover 2>&1
    python ./waf sub 2>&1
) | tee artifacts/build.txt

# copy both with exe and without to cope with differences
# between windows versions in CI
cp -v build/sitl/bin/arduplane artifacts/ArduPlane.elf.exe
cp -v build/sitl/bin/arducopter artifacts/ArduCopter.elf.exe
cp -v build/sitl/bin/arducopter-heli artifacts/ArduHeli.elf.exe
cp -v build/sitl/bin/ardurover artifacts/ArduRover.elf.exe
cp -v build/sitl/bin/ardusub artifacts/ArduSub.elf.exe

cp -v build/sitl/bin/arduplane artifacts/ArduPlane.elf
cp -v build/sitl/bin/arducopter artifacts/ArduCopter.elf
cp -v build/sitl/bin/arducopter-heli artifacts/ArduHeli.elf
cp -v build/sitl/bin/ardurover artifacts/ArduRover.elf
cp -v build/sitl/bin/ardusub artifacts/ArduSub.elf

# Find all cyg*.dll files returned by cygcheck for each exe in artifacts
# and copy them over
for exe in artifacts/*.exe; do 
    echo $exe
    cygcheck $exe | grep -oP 'cyg[^\s\\/]+\.dll' | while read -r line; do
      cp -v /usr/bin/$line artifacts/
    done
done

git log -1 > artifacts/git.txt
ls -l artifacts/

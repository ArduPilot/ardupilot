#!/bin/bash
# script to build 32 bit cygwin binaries for SITL

export PATH="/usr/local/bin:/usr/bin:/bin"
export PATH

set -x

cd /cygdrive/c/work

# build for 32 bit target
CXX=i686-pc-cygwin-g++.exe CC=i686-pc-cygwin-gcc ./waf configure --board sitl

(
    date
    git submodule update --init --recursive -f
    /usr/bin/python waf configure --board sitl
    /usr/bin/python waf -j4 copter plane rover heli sub

    # map to the names that MissionPlanner expects
    cp /cygdrive/c/work/build/sitl/bin/ardurover.exe /cygdrive/c/work/sitl/APMrover2.elf
    cp /cygdrive/c/work/build/sitl/bin/arduplane.exe /cygdrive/c/work/sitl/ArduPlane.elf
    cp /cygdrive/c/work/build/sitl/bin/arducopter.exe /cygdrive/c/work/sitl/ArduCopter.elf
    cp /cygdrive/c/work/build/sitl/bin/arducopter-heli.exe /cygdrive/c/work/sitl/ArduHeli.elf
    cp /cygdrive/c/work/build/sitl/bin/ardusub.exe /cygdrive/c/work/sitl/ArduSub.elf

    cp /usr/i686-pc-cygwin/sys-root/usr/bin/*.dll /cygdrive/c/work/sitl/

    cd /cygdrive/c/work/sitl/
    git log -1 > git.txt
    ls
) > /cygdrive/c/work/sitl/build.txt 2>&1



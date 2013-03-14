#!/bin/bash
# script to build the latest binaries for each vehicle type, ready to upload
# Andrew Tridgell, March 2013

set -e
set -x

export PATH=$PATH:/bin:/usr/bin

export TMPDIR=$PWD/build.tmp.$$
echo $TMDIR
rm -rf $TMPDIR
echo "Building in $TMPDIR"

githash=$(git rev-parse HEAD)

hdate=$(date +"%Y-%m/%Y-%m-%d-%H:%m")
mkdir -p binaries/$hdate
binaries=$PWD/../buildlogs/binaries

copyit() {
    file="$1"
    dir="$2"
    bname=$(basename $dir)
    ldir=$(dirname $(dirname $(dirname $dir)))/latest/$bname
    mkdir -p "$dir"
    /bin/cp "$file" "$dir"
    echo "$githash" > "$dir/git-version.txt"
    mkdir -p "$ldir"
    rsync "$file" "$ldir"
}

echo "Building ArduPlane binaries"
pushd ArduPlane
for b in apm1 apm2 apm1-hilsensors apm2-hilsensors apm1-1280; do
    pwd
    make clean
    make $b -j4
    copyit $TMPDIR/ArduPlane.build/ArduPlane.hex $binaries/Plane/$hdate/$b
done
popd

echo "Building ArduCopter binaries"
pushd ArduCopter
for b in apm1 apm2 apm1-hil apm2-hil; do
    pwd
    make clean
    make $b -j4
    copyit $TMPDIR/ArduCopter.build/ArduCopter.hex $binaries/Copter/$hdate/$b
done
popd

echo "Building APMRover2 binaries"
pushd APMrover2
for b in apm1 apm2 apm1-1280; do
    pwd
    make clean
    make $b -j4
    copyit $TMPDIR/APMrover2.build/APMrover2.hex $binaries/Rover/$hdate/$b
done
popd

. config.mk
test -n "$PX4_ROOT" && test -d "$PX4_ROOT" && {
    echo "Building ArduPlane PX4 binaries"
    pushd ArduPlane
    make px4-clean
    make px4
    copyit $PX4_ROOT/Images/px4fmu.px4 $binaries/Plane/$hdate/PX4
    popd

    echo "Building ArduCopter PX4 binaries"
    pushd ArduCopter
    make px4-clean
    make px4
    copyit $PX4_ROOT/Images/px4fmu.px4 $binaries/Copter/$hdate/PX4
    popd

    echo "Building APMrover2 PX4 binaries"
    pushd APMrover2
    make px4-clean
    make px4
    copyit $PX4_ROOT/Images/px4fmu.px4 $binaries/Rover/$hdate/PX4
    popd

    echo "Building PX4IO firmware"
    pushd $PX4_ROOT
    make clean
    make configure_px4io
    make
    copyit $PX4_ROOT/Images/px4io.bin $binaries/PX4IO/$hdate/PX4IO
    make configure_px4fmu
    popd
}

rm -rf $TMPDIR

exit 0

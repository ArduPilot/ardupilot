#!/bin/bash
# script to build developer releases.
# Andrew Tridgell, October 2014

set -e
set -x

export PATH=$PATH:/bin:/usr/bin

DEVBUILD="$PWD/../buildlogs/binaries/devbuild"

error_count=0

# build one developer release
build_devrelease() {
    GITURL="$1"
    BRANCH="$2"
    BUILD_TARGET="$3"
    BUILD_DIR="$4"
    RELEASE_DIR="$5"
    RELEASE_FILE="$6"

    export TMPDIR=$PWD/build.tmp.$$
    echo $TMDIR
    rm -rf $TMPDIR
    echo "Building $RELEASE_DIR in $TMPDIR"

    LOCAL_BRANCH="branch_$RELEASE_DIR"
    REMOTE_NAME="remote_$RELEASE_DIR"

    date
    git checkout "$LOCAL_BRANCH" || {
        git remote add "$REMOTE_NAME" "$GITURL" || return 1
        git fetch "$REMOTE_NAME" || return 1
        git checkout -b "$LOCAL_BRANCH" "$REMOTE_NAME/$BRANCH" -t || return 1
    }
    git fetch "$REMOTE_NAME" || return 1
    git reset --hard "$REMOTE_NAME/$BRANCH" || return 1

    echo "Checkout master for PX4Firmware"
    (cd ../PX4Firmware && git checkout master) || return 1

    echo "Checkout master for PX4NuttX"
    (cd ../PX4NuttX && git checkout master) || return 1

    mkdir -p "$DEVBUILD/$RELEASE_DIR" || return 1

    # check if we should skip this build because we have already
    # built this version
    oldversion=$(cat "$DEVBUILD/$RELEASE_DIR/git-version.txt" | head -1)
    newversion=$(git log -1 | head -1)
    [ "$oldversion" = "$newversion" ] && {
        echo "Skipping build - version match $newversion"
        return 0
    }

    pushd "$BUILD_DIR" || return 1
    make configure || return 1
    make px4-clean || return 1
    make clean || return 1
    make "$BUILD_TARGET" || return 1
    /bin/cp "$RELEASE_FILE" "$DEVBUILD/$RELEASE_DIR" || return 1
    git log -1 > "$DEVBUILD/$RELEASE_DIR/git-version.txt" || return 1
    [ -f APM_Config.h ] && {
        version=$(grep 'define.THISFIRMWARE' *.pde 2> /dev/null | cut -d'"' -f2)
        echo >> "$DEVBUILD/$RELEASE_DIR/git-version.txt"
        echo "APMVERSION: $version" >> "$DEVBUILD/$RELEASE_DIR/git-version.txt"
    }
    git checkout master || return 1
    popd

    rm -rf $TMPDIR
}

# list all developer releases here
build_devrelease git://github.com/jschall/ardupilot devbuild-jon-copter px4-v2 ArduCopter devbuild-jon-copter ArduCopter-v2.px4 || error_count=$((error_count+1))

git checkout master || error_count=$((error_count+1))

exit $error_count

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
    shift 6

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

    pushd "$BUILD_DIR" || return 1
    for frame in $*; do
        if [ -z $frame ]; then
            SUBDIR="$DEVBUILD/$RELEASE_DIR"
            make_target="$BUILD_TARGET"
        else
            SUBDIR="$DEVBUILD/$RELEASE_DIR/$frame"
            make_target="$BUILD_TARGET-$frame"
        fi

        # check if we should skip this build because we have already
        # built this version
        oldversion=$(cat "$SUBDIR/git-version.txt" | head -1)
        newversion=$(git log -1 | head -1)
        [ "$oldversion" = "$newversion" ] && {
            echo "Skipping build of $frame - version match $newversion"
            continue
        }

        mkdir -p $SUBDIR || return 1
        make px4-clean || return 1
        make clean || return 1
        make "$make_target" || return 1
        /bin/cp "$RELEASE_FILE" "$SUBDIR" || return 1
        git log -1 > "$SUBDIR/git-version.txt" || return 1
        [ -f APM_Config.h ] && {
            shopt -s nullglob
            version=$(grep 'define.THISFIRMWARE' version.h 2> /dev/null | cut -d'"' -f2)
            echo >> "$SUBDIR/git-version.txt"
            echo "APMVERSION: $version" >> "$SUBDIR/git-version.txt"
        }
    done
    git checkout master || return 1
    popd

    rm -rf $TMPDIR
}

# list all developer releases here
COPTER_FRAMES="quad tri hexa y6 octa octa-quad heli"
build_devrelease git://github.com/jschall/ardupilot devbuild-jon-copter px4-v2 ArduCopter devbuild-jon-copter ArduCopter-v2.px4 $COPTER_FRAMES || error_count=$((error_count+1))

git checkout master || error_count=$((error_count+1))

exit $error_count

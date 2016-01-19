#!/bin/bash
# script to build the latest binaries for each vehicle type, ready to upload
# Andrew Tridgell, March 2013

export PATH=$PATH:/bin:/usr/bin

export TMPDIR=$PWD/build.tmp.$$
echo $TMDIR
rm -rf $TMPDIR
echo "Building in $TMPDIR"

date
git checkout -f master
githash=$(git rev-parse HEAD)

hdate=$(date +"%Y-%m/%Y-%m-%d-%H:%m")
mkdir -p binaries/$hdate
binaries=$PWD/../buildlogs/binaries
BASEDIR=$PWD

error_count=0

. config.mk

board_branch() {
    board="$1"
    case $board in
        apm1|apm2)
            echo "-AVR"
            ;;
        *)
            echo ""
            ;;
    esac
}

# checkout the right version of the tree
checkout() {
    vehicle="$1"
    tag="$2"
    board="$3"
    frame="$4"
    echo "Trying checkout $vehicle $tag $board $frame"
    git stash
    if [ "$tag" = "latest" ]; then
	vtag="master"
    else
	vtag="$vehicle-$tag"
    fi

    # try frame specific tag
    if [ -n "$frame" ]; then
        vtag2="$vtag-$frame"

        git checkout -f "$vtag2" && {
            echo "Using frame specific tag $vtag2"
            [ -f $BASEDIR/.gitmodules ] && git submodule update
            git log -1
            return 0
        }
    fi

    # try board type specific branch extension
    vtag2="$vtag"$(board_branch $board)

    git checkout -f "$vtag2" && {
        echo "Using board specific tag $vtag2"
        [ -f $BASEDIR/.gitmodules ] && git submodule update
        git log -1
        return 0
    }

    git checkout -f "$vtag" && {
        echo "Using generic tag $vtag"
        [ -f $BASEDIR/.gitmodules ] && git submodule update
        git log -1
        return 0
    }

    echo "Failed to find tag for $vehicle $tag $board $frame"
    return 1
}

# check if we should skip this build because we have already
# built this version
skip_build() {
    [ "$FORCE_BUILD" = "1" ] && return 1
    tag="$1"
    ddir="$2"
    bname=$(basename $ddir)
    ldir=$(dirname $(dirname $(dirname $ddir)))/$tag/$bname
    [ -f $BASEDIR/.gitmodules ] || {
        echo "Skipping build without submodules"
        return 0
    }
    [ -d "$ldir" ] || {
	echo "$ldir doesn't exist - building"
	return 1
    }
    oldversion=$(cat "$ldir/git-version.txt" | head -1)
    newversion=$(git log -1 | head -1)
    [ "$oldversion" = "$newversion" ] && {
	echo "Skipping build - version match $newversion"
	return 0
    }
    echo "$ldir needs rebuild"
    return 1
}

addfwversion() {
    destdir="$1"
    git log -1 > "$destdir/git-version.txt"
    [ -f APM_Config.h ] && {
        shopt -s nullglob
	version=$(grep 'define.THISFIRMWARE' *.pde *.h 2> /dev/null | cut -d'"' -f2)
	echo >> "$destdir/git-version.txt"
	echo "APMVERSION: $version" >> "$destdir/git-version.txt"
    }    
}

# copy the built firmware to the right directory
copyit() {
    file="$1"
    dir="$2"
    tag="$3"
    bname=$(basename $dir)
    tdir=$(dirname $(dirname $(dirname $dir)))/$tag/$bname
    if [ "$tag" = "latest" ]; then
	mkdir -p "$dir"
	/bin/cp "$file" "$dir"
	addfwversion "$dir"
    fi
    echo "Copying $file to $tdir"
    mkdir -p "$tdir"
    addfwversion "$tdir"

    rsync "$file" "$tdir"
}

board_extension() {
    board="$1"
    case $board in
        apm1|apm2)
            echo "hex"
            ;;
        *)
            echo "elf"
            ;;
    esac
}

# build plane binaries
build_arduplane() {
    tag="$1"
    echo "Building ArduPlane $tag binaries from $(pwd)"
    pushd ArduPlane
    for b in apm1 apm2 navio pxf; do
        checkout ArduPlane $tag $b "" || {
            echo "Failed checkout of ArduPlane $b $tag"
            error_count=$((error_count+1))
            continue
        }
	echo "Building ArduPlane $b binaries"
	ddir=$binaries/Plane/$hdate/$b
	skip_build $tag $ddir && continue
	make clean || continue
	make $b -j4 || {
            echo "Failed build of ArduPlane $b $tag"
            error_count=$((error_count+1))
            continue
        }
        extension=$(board_extension $b)
	copyit $BUILDROOT/ArduPlane.$extension $ddir $tag
	touch $binaries/Plane/$tag
    done
    echo "Building ArduPlane PX4 binaries"
    ddir=$binaries/Plane/$hdate/PX4
    checkout ArduPlane $tag PX4 "" || {
        echo "Failed checkout of ArduPlane PX4 $tag"
        error_count=$((error_count+1))
        checkout ArduPlane "latest" "" ""
        popd
        return
    }
    skip_build $tag $ddir || {
	make px4 || {
            echo "Failed build of ArduPlane PX4 $tag"
            error_count=$((error_count+1))
            checkout ArduPlane "latest" "" ""
            popd
            return
        }
	copyit ArduPlane-v1.px4 $ddir $tag &&
	copyit ArduPlane-v2.px4 $ddir $tag &&
	test ! -f ArduPlane-v4.px4 || copyit ArduPlane-v4.px4 $ddir $tag
        if [ "$tag" = "latest" ]; then
	    copyit px4io-v1.bin $binaries/PX4IO/$hdate/PX4IO $tag
	    copyit px4io-v1.elf $binaries/PX4IO/$hdate/PX4IO $tag
	    copyit px4io-v2.bin $binaries/PX4IO/$hdate/PX4IO $tag
	    copyit px4io-v2.elf $binaries/PX4IO/$hdate/PX4IO $tag
        fi
    }
    checkout ArduPlane "latest" "" ""
    popd
}

# build copter binaries
build_arducopter() {
    tag="$1"
    echo "Building ArduCopter $tag binaries from $(pwd)"
    pushd ArduCopter
    frames="quad tri hexa y6 octa octa-quad heli"
    for b in navio pxf; do
        for f in $frames; do
            checkout ArduCopter $tag $b $f || {
                echo "Failed checkout of ArduCopter $b $tag $f"
                error_count=$((error_count+1))
                continue
            }
	    echo "Building ArduCopter $b binaries $f"
	    ddir=$binaries/Copter/$hdate/$b-$f
	    skip_build $tag $ddir && continue
	    make clean || continue
	    make $b-$f -j4 || {
                echo "Failed build of ArduCopter $b-$f $tag"
                error_count=$((error_count+1))
                continue
            }
            extension=$(board_extension $b)
	    copyit $BUILDROOT/ArduCopter.$extension $ddir $tag
	    touch $binaries/Copter/$tag
        done
    done
    for f in $frames; do
        checkout ArduCopter $tag PX4 $f || {
            echo "Failed checkout of ArduCopter PX4 $tag $f"
            error_count=$((error_count+1))
            checkout ArduCopter "latest" "" ""
            continue
        }
        rm -rf ../Build.ArduCopter
	echo "Building ArduCopter PX4-$f binaries"
	ddir="$binaries/Copter/$hdate/PX4-$f"
	skip_build $tag $ddir && continue
	make px4-$f || {
            echo "Failed build of ArduCopter PX4 $tag"
            error_count=$((error_count+1))
            continue
        }
	copyit ArduCopter-v1.px4 $ddir $tag &&
	copyit ArduCopter-v2.px4 $ddir $tag &&
	test ! -f ArduCopter-v4.px4 || copyit ArduCopter-v4.px4 $ddir $tag
    done
    checkout ArduCopter "latest" "" ""
    popd
}

# build rover binaries
build_rover() {
    tag="$1"
    echo "Building APMrover2 $tag binaries from $(pwd)"
    pushd APMrover2
    for b in apm1 apm2 navio pxf; do
	echo "Building APMrover2 $b binaries"
        checkout APMrover2 $tag $b "" || continue
	ddir=$binaries/Rover/$hdate/$b
	skip_build $tag $ddir && continue
	make clean || continue
	make $b -j4 || {
            echo "Failed build of APMrover2 $b $tag"
            error_count=$((error_count+1))
            continue
        }
        extension=$(board_extension $b)
	copyit $BUILDROOT/APMrover2.$extension $ddir $tag
	touch $binaries/Rover/$tag
    done
    echo "Building APMrover2 PX4 binaries"
    ddir=$binaries/Rover/$hdate/PX4
    checkout APMrover2 $tag PX4 "" || {
        checkout APMrover2 "latest" "" ""
        popd
        return
    }
    skip_build $tag $ddir || {
	make px4 || {
            echo "Failed build of APMrover2 PX4 $tag"
            error_count=$((error_count+1))
            checkout APMrover2 "latest" "" ""
            popd
            return
        }
	copyit APMrover2-v1.px4 $binaries/Rover/$hdate/PX4 $tag &&
	copyit APMrover2-v2.px4 $binaries/Rover/$hdate/PX4 $tag &&
	test ! -f APMrover2-v4.px4 || copyit APMrover2-v4.px4 $binaries/Rover/$hdate/PX4 $tag 
    }
    checkout APMrover2 "latest" "" ""
    popd
}

# build antenna tracker binaries
build_antennatracker() {
    tag="$1"
    echo "Building AntennaTracker $tag binaries from $(pwd)"
    pushd AntennaTracker
    for b in apm2; do
	echo "Building AntennaTracker $b binaries"
        checkout AntennaTracker $tag $b "" || continue
	ddir=$binaries/AntennaTracker/$hdate/$b
	skip_build $tag $ddir && continue
	make clean || continue
	make $b -j4 || {
            echo "Failed build of AntennaTracker $b $tag"
            error_count=$((error_count+1))
            continue
        }
        extension=$(board_extension $b)
	copyit $BUILDROOT/AntennaTracker.$extension $ddir $tag
	touch $binaries/AntennaTracker/$tag
    done
    echo "Building AntennaTracker PX4 binaries"
    ddir=$binaries/AntennaTracker/$hdate/PX4
    checkout AntennaTracker $tag PX4 "" || {
        checkout AntennaTracker "latest" "" ""
        popd
        return
    }
    skip_build $tag $ddir || {
	make px4 || {
            echo "Failed build of AntennaTracker PX4 $tag"
            error_count=$((error_count+1))
            checkout AntennaTracker "latest" "" ""
            popd
            return
        }
	copyit AntennaTracker-v1.px4 $binaries/AntennaTracker/$hdate/PX4 $tag &&
	copyit AntennaTracker-v2.px4 $binaries/AntennaTracker/$hdate/PX4 $tag &&
	test ! -f AntennaTracker-v4.px4 || copyit AntennaTracker-v4.px4 $binaries/AntennaTracker/$hdate/PX4 $tag 
    }
    checkout AntennaTracker "latest" "" ""
    popd
}

[ -f .gitmodules ] && {
    git submodule init
    git submodule update
}

export BUILDROOT="$TMPDIR/binaries.build"
rm -rf $BUILDROOT

# make sure PX4 is rebuilt from scratch
for d in ArduPlane ArduCopter APMrover2 AntennaTracker; do
         pushd $d
         make px4-clean || exit 1
         popd
done

for build in stable beta latest; do
    build_arduplane $build
    build_arducopter $build
    build_rover $build
    build_antennatracker $build
done

rm -rf $TMPDIR

exit $error_count

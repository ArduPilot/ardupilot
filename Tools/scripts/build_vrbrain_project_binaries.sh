#!/bin/bash
# script to build the latest binaries for each vehicle type, ready to upload
# Andrew Tridgell, March 2013

export PATH=$PATH:/bin:/usr/bin

export TMPDIR=$PWD/build.tmp.$$
echo $TMDIR
rm -rf $TMPDIR
echo "Building in $TMPDIR"

date
#git checkout master
#githash=$(git rev-parse HEAD)

hdate=$(date +"%Y-%m/%Y-%m-%d-%H:%m")
mkdir -p binaries/$hdate
binaries=$PWD/../buildlogs/binaries

error_count=0

. config.mk

# checkout the right version of the tree
checkout() {
    vehicle="$1"
    tag="$2"
    git stash
    if [ "$tag" = "latest" ]; then
	vbranch="for_build"
	vbranch2="for_build"
    else
	vbranch="$vehicle-$tag"
	vbranch2="for_build"
    fi

    echo "Checkout with branch $branch"

    git remote update
    git checkout -B "$vbranch" remotes/origin/"$vbranch"
    git pull -v --progress  "origin" "$vbranch" || return 1

    git log -1

    pushd ../../VRNuttX
    git remote update
    git checkout -B "$vbranch2" remotes/origin/"$vbranch2"
    git pull -v --progress  "origin" "$vbranch2" || {
        popd
        return 1
    }
    git log -1
    popd

    return 0
}

# check if we should skip this build because we have already built this version
skip_build() {
    [ "$FORCE_BUILD" = "1" ] && return 1
    tag="$1"
    ddir="$2"
    bname=$(basename $ddir)
    ldir=$(dirname $(dirname $(dirname $ddir)))/$tag/$bname
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
	version=$(grep 'define.THISFIRMWARE' version.h 2> /dev/null | cut -d'"' -f2)
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

# build plane binaries
build_arduplane() {
    tag="$1"
    echo "Building ArduPlane $tag binaries"
    pushd ArduPlane
    test -n "$VRBRAIN_ROOT" && {
	echo "Building ArduPlane VRBRAIN binaries"
	ddir=$binaries/Plane/$hdate/VRX
	skip_build $tag $ddir || {
	    make vrbrain-clean &&
	    make vrbrain || {
                echo "Failed build of ArduPlane VRBRAIN $tag"
                error_count=$((error_count+1))
                return
            }
	    copyit ArduPlane-vrbrain-v45.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v45.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v45.hex $ddir $tag &&
	    copyit ArduPlane-vrbrain-v45P.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v45P.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v45P.hex $ddir $tag &&
	    copyit ArduPlane-vrbrain-v51.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v51.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v51.hex $ddir $tag &&
	    copyit ArduPlane-vrbrain-v51P.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v51P.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v51P.hex $ddir $tag &&
	    copyit ArduPlane-vrbrain-v51Pro.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v51Pro.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v51Pro.hex $ddir $tag &&
	    copyit ArduPlane-vrbrain-v51ProP.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v51ProP.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v51ProP.hex $ddir $tag &&
	    copyit ArduPlane-vrbrain-v52.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v52.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v52.hex $ddir $tag &&
	    copyit ArduPlane-vrbrain-v52P.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v52P.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v52P.hex $ddir $tag &&
	    copyit ArduPlane-vrbrain-v52Pro.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v52Pro.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v52Pro.hex $ddir $tag &&
	    copyit ArduPlane-vrbrain-v52ProP.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v52ProP.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v52ProP.hex $ddir $tag &&
	    copyit ArduPlane-vrubrain-v51.vrx $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51.bin $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51.hex $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51P.vrx $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51P.bin $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51P.hex $ddir $tag &&
	    copyit ArduPlane-vrubrain-v52.vrx $ddir $tag && 
	    copyit ArduPlane-vrubrain-v52.bin $ddir $tag && 
	    copyit ArduPlane-vrubrain-v52.hex $ddir $tag && 
	    copyit ArduPlane-vrubrain-v52P.vrx $ddir $tag && 
	    copyit ArduPlane-vrubrain-v52P.bin $ddir $tag && 
	    copyit ArduPlane-vrubrain-v52P.hex $ddir $tag
	}
    }
}

# build copter binaries
build_arducopter() {
    tag="$1"
    echo "Building ArduCopter $tag binaries from $(pwd)"
    pushd ArduCopter
    frames="quad tri hexa y6 octa octa-quad heli"
    test -n "$VRBRAIN_ROOT"
	make vrbrain-clean || return
	for f in $frames; do
	    echo "Building ArduCopter VRBRAIN-$f binaries"
	    ddir="$binaries/Copter/$hdate/VRX-$f"
	    skip_build $tag $ddir && continue
            rm -rf ../Build.ArduCopter
	    make vrbrain-$f || {
                echo "Failed build of ArduCopter VRBRAIN $tag"
                error_count=$((error_count+1))
                continue
            }
	    copyit ArduCopter-vrbrain-v45.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v45.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v45.hex $ddir $tag &&
	    copyit ArduCopter-vrbrain-v45P.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v45P.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v45P.hex $ddir $tag &&
	    copyit ArduCopter-vrbrain-v51.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v51.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v51.hex $ddir $tag &&
	    copyit ArduCopter-vrbrain-v51P.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v51P.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v51P.hex $ddir $tag &&
	    copyit ArduCopter-vrbrain-v51Pro.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v51Pro.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v51Pro.hex $ddir $tag &&
	    copyit ArduCopter-vrbrain-v51ProP.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v51ProP.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v51ProP.hex $ddir $tag &&
	    copyit ArduCopter-vrbrain-v52.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v52.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v52.hex $ddir $tag &&
	    copyit ArduCopter-vrbrain-v52P.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v52P.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v52P.hex $ddir $tag &&
	    copyit ArduCopter-vrbrain-v52Pro.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v52Pro.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v52Pro.hex $ddir $tag &&
	    copyit ArduCopter-vrbrain-v52ProP.vrx $ddir $tag && 
	    copyit ArduCopter-vrbrain-v52ProP.bin $ddir $tag && 
	    copyit ArduCopter-vrbrain-v52ProP.hex $ddir $tag &&
	    copyit ArduCopter-vrubrain-v51.vrx $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51.bin $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51.hex $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51P.vrx $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51P.bin $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51P.hex $ddir $tag &&
	    copyit ArduCopter-vrubrain-v52.vrx $ddir $tag && 
	    copyit ArduCopter-vrubrain-v52.bin $ddir $tag && 
	    copyit ArduCopter-vrubrain-v52.hex $ddir $tag && 
	    copyit ArduCopter-vrubrain-v52P.vrx $ddir $tag && 
	    copyit ArduCopter-vrubrain-v52P.bin $ddir $tag && 
	    copyit ArduCopter-vrubrain-v52P.hex $ddir $tag
	done
    }

# build rover binaries
build_rover() {
    tag="$1"
    echo "Building APMrover2 $tag binaries from $(pwd)"
    pushd APMrover2
    test -n "$VRBRAIN_ROOT" && {
	echo "Building APMrover2 VRBRAIN binaries"
	ddir=$binaries/Rover/$hdate/VRX
	skip_build $tag $ddir || {
	    make vrbrain-clean &&
	    make vrbrain || {
                echo "Failed build of APMrover2 VRBRAIN $tag"
                error_count=$((error_count+1))
                return
            }
	    copyit APMrover2-vrbrain-v45.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v45.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v45.hex $ddir $tag &&
	    copyit APMrover2-vrbrain-v45P.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v45P.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v45P.hex $ddir $tag &&
	    copyit APMrover2-vrbrain-v51.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v51.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v51.hex $ddir $tag &&
	    copyit APMrover2-vrbrain-v51P.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v51P.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v51P.hex $ddir $tag &&
	    copyit APMrover2-vrbrain-v51Pro.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v51Pro.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v51Pro.hex $ddir $tag &&
	    copyit APMrover2-vrbrain-v51ProP.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v51ProP.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v51ProP.hex $ddir $tag &&
	    copyit APMrover2-vrbrain-v52.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v52.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v52.hex $ddir $tag &&
	    copyit APMrover2-vrbrain-v52P.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v52P.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v52P.hex $ddir $tag &&
	    copyit APMrover2-vrbrain-v52Pro.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v52Pro.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v52Pro.hex $ddir $tag &&
	    copyit APMrover2-vrbrain-v52ProP.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v52ProP.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v52ProP.hex $ddir $tag &&
	    copyit APMrover2-vrubrain-v51.vrx $ddir $tag && 
	    copyit APMrover2-vrubrain-v51.bin $ddir $tag && 
	    copyit APMrover2-vrubrain-v51.hex $ddir $tag && 
	    copyit APMrover2-vrubrain-v51P.vrx $ddir $tag && 
	    copyit APMrover2-vrubrain-v51P.bin $ddir $tag && 
	    copyit APMrover2-vrubrain-v51P.hex $ddir $tag &&
	    copyit APMrover2-vrubrain-v52.vrx $ddir $tag && 
	    copyit APMrover2-vrubrain-v52.bin $ddir $tag && 
	    copyit APMrover2-vrubrain-v52.hex $ddir $tag && 
	    copyit APMrover2-vrubrain-v52P.vrx $ddir $tag && 
	    copyit APMrover2-vrubrain-v52P.bin $ddir $tag && 
	    copyit APMrover2-vrubrain-v52P.hex $ddir $tag
	}
    }
}

for build in latest; do
    build_arduplane $build
    build_arducopter $build
    build_rover $build
done

rm -rf $TMPDIR

exit $error_count

#!/bin/bash
# script to build the latest binaries for each vehicle type, ready to upload
# Andrew Tridgell, March 2013

export PATH=$PATH:/bin:/usr/bin

export TMPDIR=$PWD/build.tmp.$$
echo $TMDIR
rm -rf $TMPDIR
echo "Building in $TMPDIR"

date
git checkout master
githash=$(git rev-parse HEAD)

hdate=$(date +"%Y-%m/%Y-%m-%d-%H:%m")
mkdir -p binaries/$hdate
binaries=$PWD/../buildlogs/binaries

error_count=0

. config.mk

# checkout the right version of the tree
checkout() {
    branch="$1"
    git stash
    if [ "$branch" = "master" ]; then
	vbranch="master"
	vbranch2="master"
    fi
    if [ "$branch" = "for_merge" ]; then
	vbranch="for_merge"
	vbranch2="for_merge"
    fi
    if [ "$branch" = "for_merge-3.2" ]; then
	vbranch="for_merge-3.2"
	vbranch2="for_merge"
    fi
    if [ "$branch" = "tone_alarm" ]; then
	vbranch="ToneAlarm"
	vbranch2="ToneAlarm"
    fi
    if [ "$branch" = "tone_alarm-3.2" ]; then
	vbranch="ToneAlarm-3.2"
	vbranch2="ToneAlarm"
    fi

    echo "Checkout with branch $branch"

    git remote update
    git checkout -B "$vbranch" remotes/origin/"$vbranch"
    git pull -v --progress  "origin" "$vbranch" || return 1

    git log -1

    pushd ../../vrbrain_nuttx
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

# check if we should skip this build because we have already
# built this version
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
	version=$(grep 'define.THISFIRMWARE' *.pde 2> /dev/null | cut -d'"' -f2)
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
        checkout $tag || {
            echo "Failed checkout of ArduPlane VRBRAIN $tag"
            error_count=$((error_count+1))
            continue
        }
	skip_build $tag $ddir || {
	    make vrbrain-clean &&
	    make vrbrain || {
                echo "Failed build of ArduPlane VRBRAIN $tag"
                error_count=$((error_count+1))
                continue
            }
	    copyit ArduPlane-vrbrain-v45.vrx $ddir $tag && 
	    copyit ArduPlane-vrbrain-v45.bin $ddir $tag && 
	    copyit ArduPlane-vrbrain-v45.hex $ddir $tag &&
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
	    copyit ArduPlane-vrubrain-v51.vrx $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51.bin $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51.hex $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51P.vrx $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51P.bin $ddir $tag && 
	    copyit ArduPlane-vrubrain-v51P.hex $ddir $tag
	}
    }
    checkout "master"
    popd
}

# build copter binaries
build_arducopter() {
    tag="$1"
    echo "Building ArduCopter $tag binaries from $(pwd)"
    pushd ArduCopter
    frames="quad tri hexa y6 octa octa-quad heli"
    test -n "$VRBRAIN_ROOT" && {
        checkout $tag || {
            echo "Failed checkout of ArduCopter VRBRAIN $tag"
            error_count=$((error_count+1))
            checkout "master"
            popd
            return
        }
	make vrbrain-clean || return
	for f in $frames quad-hil heli-hil; do
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
	    copyit ArduCopter-vrubrain-v51.vrx $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51.bin $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51.hex $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51P.vrx $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51P.bin $ddir $tag && 
	    copyit ArduCopter-vrubrain-v51P.hex $ddir $tag
	done
    }
    checkout "master"
    popd
}

# build rover binaries
build_rover() {
    tag="$1"
    echo "Building APMrover2 $tag binaries from $(pwd)"
    pushd APMrover2
    test -n "$VRBRAIN_ROOT" && {
	echo "Building APMrover2 VRBRAIN binaries"
	ddir=$binaries/Rover/$hdate/VRX
        checkout $tag || {
            checkout "master"
            popd
            return
        }
	skip_build $tag $ddir || {
	    make vrbrain-clean &&
	    make vrbrain || {
                echo "Failed build of APMrover2 VRBRAIN $tag"
                error_count=$((error_count+1))
                checkout APMrover2 "latest" ""
                popd
                return
            }
	    copyit APMrover2-vrbrain-v45.vrx $ddir $tag && 
	    copyit APMrover2-vrbrain-v45.bin $ddir $tag && 
	    copyit APMrover2-vrbrain-v45.hex $ddir $tag &&
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
	    copyit APMrover2-vrubrain-v51.vrx $ddir $tag && 
	    copyit APMrover2-vrubrain-v51.bin $ddir $tag && 
	    copyit APMrover2-vrubrain-v51.hex $ddir $tag && 
	    copyit APMrover2-vrubrain-v51P.vrx $ddir $tag && 
	    copyit APMrover2-vrubrain-v51P.bin $ddir $tag && 
	    copyit APMrover2-vrubrain-v51P.hex $ddir $tag
	}
    }
    checkout "master"
    popd
}

for build in for_merge for_merge-3.2 tone_alarm tone_alarm-3.2; do
    build_arduplane $build
    build_arducopter $build
    build_rover $build
done

rm -rf $TMPDIR

exit $error_count

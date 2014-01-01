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

. config.mk

# checkout the right version of the tree
checkout() {
    vehicle="$1"
    tag="$2"
    git stash
    if [ "$tag" = "latest" ]; then
	git checkout master || return 1
    else
	git checkout "$vehicle-$tag" || return 1
    fi
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
    checkout ArduPlane $tag || return
    pushd ArduPlane
    for b in apm1 apm2 apm1-hilsensors apm2-hilsensors; do
	echo "Building ArduPlane $b binaries"
	ddir=$binaries/Plane/$hdate/$b
	skip_build $tag $ddir && continue
	make clean || continue
	make $b -j4 || continue
	copyit $TMPDIR/ArduPlane.build/ArduPlane.hex $ddir $tag
	touch $binaries/Plane/$tag
    done
    test -n "$PX4_ROOT" && {
	echo "Building ArduPlane PX4 binaries"
	ddir=$binaries/Plane/$hdate/PX4
	skip_build $tag $ddir || {
	    make px4-clean &&
	    make px4 &&
	    copyit ArduPlane-v1.px4 $ddir $tag &&
	    copyit ArduPlane-v2.px4 $ddir $tag
            if [ "$tag" = "latest" ]; then
	        copyit px4io-v1.bin $binaries/PX4IO/$hdate/PX4IO $tag
	        copyit px4io-v1.elf $binaries/PX4IO/$hdate/PX4IO $tag
	        copyit px4io-v2.bin $binaries/PX4IO/$hdate/PX4IO $tag
	        copyit px4io-v2.elf $binaries/PX4IO/$hdate/PX4IO $tag
            fi
	}
    }
    popd
    git checkout master
}

# build copter binaries
build_arducopter() {
    tag="$1"
    checkout ArduCopter $tag || return
    echo "Building ArduCopter $tag binaries"
    pushd ArduCopter
    frames="quad tri hexa y6 octa octa-quad heli"
    for b in apm1 apm2; do
	for f in $frames quad-hil heli-hil; do
	    echo "Building ArduCopter $b-$f binaries"
	    ddir="$binaries/Copter/$hdate/$b-$f"
	    skip_build $tag $ddir && continue
	    make clean || continue
	    make "$b-$f" -j4 || exit 1
	    copyit $TMPDIR/ArduCopter.build/ArduCopter.hex "$ddir" "$tag"
	    touch $binaries/Copter/$tag
	done
    done
    test -n "$PX4_ROOT" && {
	make px4-clean || return
	for f in $frames quad-hil heli-hil; do
	    echo "Building ArduCopter PX4-$f binaries"
	    ddir="$binaries/Copter/$hdate/PX4-$f"
	    skip_build $tag $ddir && continue
            rm -rf ../Build.ArduCopter
	    make px4-$f || continue
	    copyit ArduCopter-v1.px4 $ddir $tag &&
	    copyit ArduCopter-v2.px4 $ddir $tag
	done
    }
    popd
    git checkout master
}

# build rover binaries
build_rover() {
    tag="$1"
    checkout APMrover2 $tag || return
    echo "Building APMrover2 $tag binaries"
    pushd APMrover2
    for b in apm1 apm2 apm1-1280; do
	echo "Building APMrover2 $b binaries"
	ddir=$binaries/Rover/$hdate/$b
	skip_build $tag $ddir && continue
	make clean || continue
	make $b -j4 || continue
	copyit $TMPDIR/APMrover2.build/APMrover2.hex $ddir $tag
	touch $binaries/Rover/$tag
    done
    test -n "$PX4_ROOT" && {
	echo "Building APMrover2 PX4 binaries"
	ddir=$binaries/Rover/$hdate/PX4
	skip_build $tag $ddir || {
	    make px4-clean &&
	    make px4 &&
	    copyit APMrover2-v1.px4 $binaries/Rover/$hdate/PX4 $tag &&
	    copyit APMrover2-v2.px4 $binaries/Rover/$hdate/PX4 $tag 
	}
    }
    popd
    git checkout master
}

for build in stable beta latest; do
    build_arduplane $build
    build_arducopter $build
    build_rover $build
done

rm -rf $TMPDIR

exit 0

#!/bin/bash

############################
# grab a lock file. Not atomic, but close :)
# tries to cope with NFS
lock_file() {
        lck="$1"
        pid=`cat "$lck" 2> /dev/null`

        if test -f "$lck" && kill -0 $pid 2> /dev/null; then
	    LOCKAGE=$(($(date +%s) - $(stat -c '%Y' "build.lck")))
	    test $LOCKAGE -gt 80000 && {
                echo "old lock file $lck is valid for $pid with age $LOCKAGE seconds"
	    }
            return 1
        fi
        /bin/rm -f "$lck"
        echo "$$" > "$lck"
        return 0
}


lock_file sitl_dl.lck || {
    exit 1
}

DEST="$HOME/APM/buildlogs/binaries/Tools/MissionPlanner/sitl"

function download() {
    branch="$1"
    dirname="$2"
    rm -f binaries.zip
    rm -f wget*
    wget -q https://nightly.link/ArduPilot/ardupilot/workflows/cygwin_build/$branch/binaries.zip || true
    [ -f binaries.zip ] || {
      echo "Download of $branch failed"
      return
    }
    zfile="sitl-$branch.zip"
    if cmp $zfile binaries.zip; then
	echo "No change in $branch"
	return
    fi
    rm -f $zfile
    mv binaries.zip $zfile
    unzip -o $zfile
    [ -f 'ArduCopter.elf.exe' ] && {
	for f in ArduPlane ArduCopter ArduSub ArduRover ArduHeli; do
	    [ -f $f.elf.exe ] && mv $f.elf.exe $f.elf
	done
	[ -f ArduRover.elf ] && cp ArduRover.elf APMrover2.elf
	if [ "$branch" = "master" ]; then
            rsync --exclude='*.zip' --exclude='wget*' -av *.elf *.dll *.txt $DEST
	else
	    mkdir -p $DEST/$dirname
            rsync --exclude='*.zip' --exclude='wget*' -av *.elf *.dll *.txt $DEST/$dirname
	fi
	if [ "$dirname" = "Stable" ]; then
	    for t in PlaneStable CopterStable RoverStable; do
		mkdir -p $DEST/$t
		rsync -av --delete $DEST/$dirname/ $DEST/$t/
	    done
	fi 
    }
}
   

(
    set -x
    set -e
    cd ~/
    echo
    date
    mkdir -p ~/sitl-download
    cd ~/sitl-download
    download master ""
    download ArduPilot-4.6 Stable
    download ArduPilot-4.6 Beta
) >> ~/sitl_dl.log 2>&1


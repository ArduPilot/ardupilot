#!/bin/bash

export PATH=$HOME/.local/bin:/usr/local/bin:$HOME/prefix/bin:$HOME/APM/px4/gcc-arm-none-eabi-4_9-2015q3/bin:$PATH
export PYTHONUNBUFFERED=1
export PYTHONPATH=$HOME/APM

cd $HOME/APM || exit 1

test -n "$FORCEBUILD" || {
(cd APM && git fetch > /dev/null 2>&1)

newtags=$(cd APM && git fetch --tags | wc -l)
oldhash=$(cd APM && git rev-parse origin/master)
newhash=$(cd APM && git rev-parse HEAD)

if [ "$oldhash" = "$newhash" -a "$newtags" = "0" ]; then
    echo "$(date) no change $oldhash $newhash" >> build.log
    exit 0
fi
echo "$(date) Build triggered $oldhash $newhash $newtags" >> build.log
}

############################
# grab a lock file. Not atomic, but close :)
# tries to cope with NFS
lock_file() {
        lck="$1"
        pid=`cat "$lck" 2> /dev/null`

        if test -f "$lck" && kill -0 $pid 2> /dev/null; then
	    LOCKAGE=$(($(date +%s) - $(stat -c '%Y' "build.lck")))
	    test $LOCKAGE -gt 60000 && {
                echo "old lock file $lck is valid for $pid with age $LOCKAGE seconds"
	    }
            return 1
        fi
        /bin/rm -f "$lck"
        echo "$$" > "$lck"
        return 0
}


lock_file build.lck || {
    exit 1
}


#ulimit -m 500000
#ulimit -s 500000
#ulimit -t 1800
#ulimit -v 500000

(
date

report() {
    d="$1"
    old="$2"
    new="$3"
    cat <<EOF | mail -s 'build failed' drones-discuss@googlegroups.com
A build of $d failed at `date`

You can view the build logs at http://autotest.ardupilot.org/

A log of the commits since the last attempted build is below

`git log $old $new`
EOF
}

report_pull_failure() {
    d="$1"
    git show origin/master | mail -s 'APM pull failed' drones-discuss@googlegroups.com
    exit 1
}

oldhash=$(cd APM && git rev-parse HEAD)

echo "Updating APM"
pushd APM
git checkout -f master
git fetch origin
git submodule update --recursive --force
git reset --hard origin/master
git pull || report_pull_failure
git clean -f -f -x -d -d
git tag autotest-$(date '+%Y-%m-%d-%H%M%S') -m "test tag `date`"
cp ../config.mk .
popd

rsync -a APM/Tools/autotest/web-firmware/ buildlogs/binaries/

echo "Updating pymavlink"
pushd APM/modules/mavlink/pymavlink
git show
python setup.py build install --user
popd

echo "Updating MAVProxy"
pushd MAVProxy
git fetch origin
git reset --hard origin/master
git show
python setup.py build install --user
popd

githash=$(cd APM && git rev-parse HEAD)
hdate=$(date +"%Y-%m-%d-%H:%m")

for d in ArduPlane ArduCopter APMrover2 AntennaTracker; do
    pushd APM/$d
    rm -rf ../../buildlogs/$d.build
    (date && TMPDIR=../../buildlogs make) > ../../buildlogs/$d.txt 2>&1
    status=$?
    if [ $status != 0 ]; then
	report $d $oldhash $newhash
    fi
    popd
    APM/Tools/scripts/frame_sizes.py buildlogs/$d.build > buildlogs/$d.framesizes.txt
    (
	avr-size buildlogs/$d.build/$d.elf 
	avr-nm --size-sort --print-size -C buildlogs/$d.build/$d.elf 
    ) > buildlogs/$d.sizes.txt
done

mkdir -p "buildlogs/history/$hdate"
(cd buildlogs && cp -f *.txt *.flashlog *.tlog *.km[lz] *.gpx *.html *.png *.bin *.BIN *.elf "history/$hdate/")
echo $githash > "buildlogs/history/$hdate/githash.txt"

(cd APM && Tools/scripts/build_parameters.sh)

(cd APM && Tools/scripts/build_docs.sh)

killall -9 JSBSim || /bin/true

# raise core limit
ulimit -c 10000000

timelimit 32000 APM/Tools/autotest/autotest.py --timeout=30000 > buildlogs/autotest-output.txt 2>&1

) >> build.log 2>&1

# autotest done, let's mark GTD flags
touch /tmp/.autotest.done


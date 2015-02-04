#!/bin/bash

export PATH=$HOME/.local/bin:/usr/local/bin:$HOME/prefix/bin:$HOME/APM/px4/gcc-arm-none-eabi-4_7-2014q2/bin:$PATH
export PYTHONUNBUFFERED=1
export PYTHONPATH=$HOME/APM

cd $HOME/APM || exit 1

test -n "$FORCEBUILD" || {
(cd APM && git fetch > /dev/null 2>&1)

newtags=$(cd APM && git fetch --tags | wc -l)
oldhash=$(cd APM && git rev-parse origin/master)
newhash=$(cd APM && git rev-parse HEAD)

newtagspx4=$(cd PX4Firmware && git fetch --tags | wc -l)
oldhashpx4=$(cd PX4Firmware && git rev-parse origin/master)
newhashpx4=$(cd PX4Firmware && git rev-parse HEAD)

newtagsnuttx=$(cd PX4NuttX && git fetch --tags | wc -l)
oldhashnuttx=$(cd PX4NuttX && git rev-parse origin/master)
newhashnuttx=$(cd PX4NuttX && git rev-parse HEAD)

newtagsuavcan=$(cd uavcan && git fetch --tags | wc -l)
oldhashuavcan=$(cd uavcan && git rev-parse origin/master)
newhashuavcan=$(cd uavcan && git rev-parse HEAD)

if [ "$oldhash" = "$newhash" -a "$newtags" = "0" -a "$oldhashpx4" = "$newhashpx4" -a "$newtagspx4" = "0" -a "$oldhashnuttx" = "$newhashnuttx" -a "$newtagsnuttx" = "0" -a "$oldhashuavcan" = "$newhashuavcan" -a "$newtagsuavcan" = "0" ]; then
    echo "no change $oldhash $newhash `date`" >> build.log
    exit 0
fi
}

############################
# grab a lock file. Not atomic, but close :)
# tries to cope with NFS
lock_file() {
        lck="$1"
        pid=`cat "$lck" 2> /dev/null`

        if test -f "$lck" && kill -0 $pid 2> /dev/null; then
	    LOCKAGE=$(($(date +%s) - $(stat -c '%Y' "build.lck")))
	    test $LOCKAGE -gt 7200 && {
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

You can view the build logs at http://autotest.diydrones.com/

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

pushd APM
git pull || report_pull_failure
git clean -f -f -x -d -d
git tag autotest-$(date '+%Y-%m-%d-%H%M%S') -m "test tag `date`"
cp ../config.mk .
popd

rsync -a APM/Tools/autotest/web-firmware/ buildlogs/binaries/

pushd PX4Firmware
git fetch origin
git reset --hard origin/master
for v in ArduPlane ArduCopter APMrover2; do
    git tag -d $v-beta || true
    git tag -d $v-stable || true
done
git fetch origin --tags
git show
popd

pushd PX4NuttX
git fetch origin
git reset --hard origin/master
for v in ArduPlane ArduCopter APMrover2; do
    git tag -d $v-beta || true
    git tag -d $v-stable || true
done
git fetch origin --tags
git show
popd

pushd uavcan
git fetch origin
git reset --hard origin/master
for v in ArduPlane ArduCopter APMrover2; do
    git tag -d $v-beta || true
    git tag -d $v-stable || true
done
git fetch origin --tags
git show
popd

echo "Updating pymavlink"
pushd mavlink/pymavlink
git fetch origin
git reset --hard origin/master
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

for d in ArduPlane ArduCopter APMrover2; do
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

timelimit 8500 APM/Tools/autotest/autotest.py --timeout=8000 > buildlogs/autotest-output.txt 2>&1

) >> build.log 2>&1

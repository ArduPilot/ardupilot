#!/usr/bin/env bash

export PATH=$HOME/.local/bin:/usr/local/bin:$HOME/prefix/bin:$HOME/gcc/active/bin:$PATH
export PYTHONUNBUFFERED=1

cd $HOME/APM || exit 1

ARDUPILOT_ROOT="$PWD/APM"

test -n "$FORCEBUILD" || {
  pushd APM
    git fetch > /dev/null 2>&1
    newtags=$(git fetch --tags --force | wc -l)
    oldhash=$(git rev-parse origin/master)
    newhash=$(git rev-parse HEAD)
  popd

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
	    test $LOCKAGE -gt 80000 && {
                echo "old lock file $lck is valid for $pid with age $LOCKAGE seconds"
	    }
            return 1
        fi
        rm -f "$lck"
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
set -x

date

echo "Updating ArduPilot repository"
pushd "$ARDUPILOT_ROOT"
git checkout -f master
git fetch origin
git reset --hard origin/master
Tools/gittools/submodule-sync.sh
git clean -f -f -x -d -d
git tag autotest-$(date '+%Y-%m-%d-%H%M%S') -m "test tag `date`"
popd

rsync -a APM/Tools/autotest/web-firmware/ buildlogs/binaries/

echo "Updating MAVProxy"
pushd MAVProxy
git fetch origin
git reset --hard origin/master
git show
python3 -m pip install --user .
popd

echo "Updating pymavlink"
pushd APM/modules/mavlink/pymavlink
git show
python3 -m pip install --user .
popd

githash=$(cd APM && git rev-parse HEAD)
hdate=$(date +"%Y-%m-%d-%H:%m")

pushd $ARDUPILOT_ROOT
Tools/scripts/build_parameters.sh
Tools/scripts/build_log_message_documentation.sh
Tools/scripts/build_docs.sh
popd

killall -9 JSBSim || /bin/true

# raise core limit
ulimit -c 10000000

# build in home dir, as on faster storage
export BUILD_BINARIES_PATH=$HOME/build/tmp

# exit on panic so we don't waste time waiting around
export SITL_PANIC_EXIT=1

# we run the timelimit shell command to kill autotest if it behaves badly:
TIMELIMIT_TIME_LIMIT=144000
# we pass this into autotest.py to get it to time limit itself
AUTOTEST_TIME_LIMIT=143000

# the autotest python script:
AUTOTEST="$ARDUPILOT_ROOT/Tools/autotest/autotest.py"

# decide which timelimit command we are working with.  The autotest
# server has a binary of unknown lineage in
# /home/autotest/bin/timelimit .  We should move to using the
# apt-installable version

if timelimit 2>&1 | grep -q SIGQUIT; then
    TIMELIMIT_CMD="timelimit $TIMELIMIT_TIME_LIMIT"
else
    TIMELIMIT_CMD="timelimit -s 9 -t $TIMELIMIT_TIME_LIMIT"
fi

AUTOTEST_LOG="buildlogs/autotest-output.txt"
echo "AutoTest log file is ($AUTOTEST_LOG)"
$TIMELIMIT_CMD python3 $AUTOTEST --autotest-server --timeout=$AUTOTEST_TIME_LIMIT > "$AUTOTEST_LOG" 2>&1

mkdir -p "buildlogs/history/$hdate"

(cd buildlogs && cp -f *.txt *.flashlog *.tlog *.km[lz] *.gpx *.html *.png *.bin *.BIN *.elf "history/$hdate/")
echo $githash > "buildlogs/history/$hdate/githash.txt"

) >> build.log 2>&1

# autotest done, let's mark GTD flags
touch /tmp/.autotest.done


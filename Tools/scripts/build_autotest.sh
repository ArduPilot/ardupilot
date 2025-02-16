#!/usr/bin/env bash

set -e
set -x

export PATH=$HOME/.local/bin:/usr/local/bin:$HOME/prefix/bin:$HOME/gcc/active/bin:$PATH
export PYTHONUNBUFFERED=1

cd $HOME/APM || exit 1

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


lock_file build.lck || {
    echo "$(date) Build locked" >> build.log
    exit 1
}

if [ -e "$HOME/APM/FORCEBUILD" ]; then
   unlink "$HOME/APM/FORCEBUILD"
   FORCEBUILD=1
fi

test -n "$FORCEBUILD" || {
pushd APM
git fetch > /dev/null 2>&1
newtags=$(git fetch --tags | wc -l)
newhash=$(git rev-parse origin/master)
oldhash=$(git rev-parse master)
popd

if [ "$oldhash" = "$newhash" -a "$newtags" = "0" ]; then
    echo "$(date) no change $oldhash $newhash" >> build.log
    exit 0
fi
echo "$(date) Build triggered $oldhash $newhash $newtags (see $PWD/build.log)" >> build.log
}

#ulimit -m 500000
#ulimit -s 500000
#ulimit -t 1800
#ulimit -v 500000

(
date

echo "Updating APM"
pushd APM
git checkout -f master
git fetch origin
git reset --hard origin/master
Tools/gittools/submodule-sync.sh
git clean -f -f -x -d -d
git tag autotest-$(date '+%Y-%m-%d-%H%M%S') -m "test tag `date`"
popd

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

WEB_BOILERPLATE="$PWD/APM/Tools/autotest/web-firmware"

killall -9 JSBSim || /bin/true

# raise core limit
ulimit -c 10000000

# build in home dir, as on faster storage
export BUILD_BINARIES_PATH=$HOME/build/tmp

# exit on panic so we don't waste time waiting around
export SITL_PANIC_EXIT=1

ARDUPILOT_ROOT="$PWD/APM"
AUTOTEST="$ARDUPILOT_ROOT/Tools/autotest/autotest.py"
export AUTOTEST_LOCKFILE="$PWD/autotest.lck"

# we run the timelimit shell command to kill autotest if it behaves badly:
TIMELIMIT_TIME_LIMIT=72000
# we pass this into autotest.py to get it to time limit itself
AUTOTEST_TIME_LIMIT=70000

if [ "x$BUILDLOGS" = "x" ]; then
    BUILDLOGS="buildlogs"
fi

mkdir -p $BUILDLOGS

pushd $BUILDLOGS

  export BUILD_BINARIES_BUILDLOGS_DIR="$PWD"
  export BUILD_BINARIES_HISTORY="$PWD/build_binaries_history.sqlite"

  HISTORY_DIR="history/$hdate"
  mkdir -p "$HISTORY_DIR"

  # populate index.html etc from the repository:
  rsync -aPH "$WEB_BOILERPLATE/" "$HISTORY_DIR"

  # create a link so people can see what we're currently doing:
  ln -sfn "$HISTORY_DIR" currently-building

  pushd "$HISTORY_DIR"

    echo $githash > "githash.txt"

    # autotest.py honours BUILDLOGS
    export BUILDLOGS="$PWD"

    TIMELIMIT=""
    if [ -n "$(which timelimit)" ]; then
        TIMELIMIT="timelimit $TIMELIMIT_TIME_LIMIT"
    fi
    AUTOTEST_LOG="$PWD/autotest-output.txt"
    echo "AutoTest log file is ($AUTOTEST_LOG)"
    $TIMELIMIT "$AUTOTEST" --autotest-server --timeout=300000 > "$AUTOTEST_LOG" 2>&1 || true  # ignore test failure
  popd

  echo "Removing logs from final successful tests"
  pushd $ARDUPILOT_ROOT
  rm -rf logs/*
  popd

  pushd $ARDUPILOT_ROOT
  ./Tools/scripts/build_parameters.sh
  ./Tools/scripts/build_log_message_documentation.sh
  ./Tools/scripts/build_docs.sh
  popd

  # copy Parameters and LogMessages from HISTORY down into top-level
  # dir.  This shouldn't be required as the web server should be
  # configured to take from the "latest" link.
  pushd "$BUILDLOGS/$HISTORY_DIR"
  for dir in "Parameters" "LogMessages"; do
      SOURCE="$dir"
      OUT="$BUILD_BINARIES_BUILDLOGS_DIR/$dir"

      rsync -aP "$SOURCE/" "$OUT"
  done
  popd

  # autotest is done, so update the link to the most-recent build
  ln -sfn "$HISTORY_DIR" latest

  # remove the "currently-building" link as we're not currently building
  rm -f currently-building
popd

) >> build.log 2>&1

# autotest done, let's mark GTD flags
touch /tmp/.autotest.done


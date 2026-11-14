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
    echo "$(date) Build locked" >> build.log
    exit 1
}


#ulimit -m 500000
#ulimit -s 500000
#ulimit -t 1800
#ulimit -v 500000

(
set -x
set -e

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
hdate=$(date +"%Y-%m-%d-%H:%M")

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

# Each run's autotest results go in their own directory under history.
# State which must persist from one run to the next (binaries, the
# build history database, the lock) stays at the top level.  The
# doxygen output from build_docs.sh stays in the git checkout.
BUILDLOGS_ROOT="$PWD/buildlogs"
export AUTOTEST_LOCKFILE="$PWD/autotest.lck"
export BUILD_BINARIES_BUILDLOGS_DIR="$BUILDLOGS_ROOT"

# refuse to reuse a directory, which would mix two runs' results:
mkdir -p "$BUILDLOGS_ROOT/history"
RUN_DIR="$BUILDLOGS_ROOT/history/$hdate"
mkdir "$RUN_DIR"
echo "$githash" > "$RUN_DIR/githash.txt"

# move a real file or directory left by the old copy-into-place
# layout out of the way of a link:
move_aside() {
    if [ -e "$1" ] && [ ! -L "$1" ]; then
        mv -T "$1" "$1.pre-links"
    fi
}

# atomically point the link NAME in BUILDLOGS_ROOT at TARGET (relative
# to BUILDLOGS_ROOT):
switch_link() {
    move_aside "$BUILDLOGS_ROOT/$1"
    ln -sfn "$2" "$BUILDLOGS_ROOT/$1.new"
    mv -T "$BUILDLOGS_ROOT/$1.new" "$BUILDLOGS_ROOT/$1"
}

# if anything below fails, set -e ends this subshell; mark the run
# rather than leaving in-progress pointing at it:
abandon_run() {
    local status=$?
    if [ "$status" -ne 0 ]; then
        echo "run aborted with status $status" > "$RUN_DIR/ABORTED"
        rm -f "$BUILDLOGS_ROOT/in-progress"
    fi
}
trap abandon_run EXIT

switch_link in-progress "history/$hdate"

# autotest.py kills its whole process group on timeout, so give it
# its own session to stop it killing this script too.
AUTOTEST_LOG="$RUN_DIR/autotest-output.txt"
echo "AutoTest log file is ($AUTOTEST_LOG)"
autotest_status=0
BUILDLOGS="$RUN_DIR" setsid --wait $TIMELIMIT_CMD python3 $AUTOTEST --autotest-server --timeout=$AUTOTEST_TIME_LIMIT > "$AUTOTEST_LOG" 2>&1 || autotest_status=$?

# autotest.py exits 0 if every test passed and 1 if any failed, but also
# 1 if it could not start or crashed.  It writes index.html once it has
# results, including when it times out itself, so publish any run which
# has one:
if [ "$autotest_status" -gt 1 ]; then
    echo "autotest did not finish (exit status $autotest_status)" > "$RUN_DIR/INCOMPLETE"
fi
if [ -e "$RUN_DIR/index.html" ]; then
    switch_link last-completed "history/$hdate"
    # keep the old top-level URLs (index.html, autotest-badge.svg, the
    # per-test files) working.  The links go through last-completed, so
    # switching that link updates them all at once:
    for f in "$RUN_DIR"/*; do
        name=$(basename "$f")
        case "$name" in
            binaries|history|Parameters|LogMessages|in-progress|last-completed) continue ;;
        esac
        move_aside "$BUILDLOGS_ROOT/$name"
        ln -sfn "last-completed/$name" "$BUILDLOGS_ROOT/$name"
    done
else
    echo "autotest wrote no results (exit status $autotest_status)" >> "$RUN_DIR/INCOMPLETE"
fi

# build_binaries.py checks out release tags; a timeout can leave the
# tree on one of those:
pushd "$ARDUPILOT_ROOT"
git checkout -f "$githash"
Tools/scripts/build_docs.sh || echo "build_docs.sh failed"

# generate into the run directory, only switching the published link
# if generation succeeds:
if BUILDLOGS="$RUN_DIR/param-generation" Tools/scripts/build_parameters.sh; then
    switch_link Parameters "history/$hdate/param-generation/Parameters"
else
    echo "parameter generation failed; Parameters is still $(readlink "$BUILDLOGS_ROOT/Parameters")" > "$RUN_DIR/PARAMETERS-NOT-PUBLISHED"
fi
if BUILDLOGS="$RUN_DIR/logmessage-generation" Tools/scripts/build_log_message_documentation.sh; then
    switch_link LogMessages "history/$hdate/logmessage-generation/LogMessages"
else
    echo "log message generation failed; LogMessages is still $(readlink "$BUILDLOGS_ROOT/LogMessages")" > "$RUN_DIR/LOGMESSAGES-NOT-PUBLISHED"
fi
popd

rm -f "$BUILDLOGS_ROOT/in-progress"

) >> build.log 2>&1
status=$?

# autotest done, let's mark GTD flags
touch /tmp/.autotest.done

exit $status


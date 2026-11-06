#!/usr/bin/env bash
# Run a SITL shared-memory cluster on the Cygwin/Windows CI runner and
# report the speedup it actually achieves.
#
# Three copters join one cluster at a commanded 100x speedup with the
# adaptive physics-rate governor on (SITL_ADAPTIVE_RATE), sit on the
# ground unarmed for a fixed wall-clock window, and are then checked
# for (a) cluster lock-step actually engaging, (b) the governor making
# at least one rate decision, and (c) a reported achieved speedup. The
# achieved figure is printed rather than asserted on: a shared 2-4 core
# runner cannot promise any particular number, but the log makes the
# Windows build's real capability visible in every CI run.
#
# Pure C++ + stdlib: the vehicles do all clustering themselves; python
# (any version, no pymavlink) only opens each vehicle's SERIAL0 TCP
# port and drains it so the vehicles never block on telemetry.

set -e

# Run the binary straight out of build/sitl/bin, NOT the artifacts
# copy: artifacts/ bundles its own cyg*.dll set for standalone use, and
# Windows resolves DLLs from the exe's directory first, so launching
# that copy from inside the workflow's installed Cygwin loads a second,
# potentially mismatched cygwin1.dll - a classic silent startup death.
# The build-tree binary has no DLLs beside it and uses the running
# Cygwin's own runtime. Also the natural path for exercising this
# script on Linux.
COPTER=${CLUSTER_SMOKE_COPTER:-build/sitl/bin/arducopter}
RUN_SECONDS=${CLUSTER_SMOKE_RUN_SECONDS:-40}
CLUSTER_ID=77
NVEHICLES=3

if [ ! -x "$COPTER" ]; then
    echo "cluster-smoke: no executable at $COPTER" >&2
    exit 1
fi
COPTER=$(cd "$(dirname "$COPTER")" && pwd)/$(basename "$COPTER")

WORK=$(mktemp -d)
trap 'kill $(jobs -p) 2>/dev/null || true; wait 2>/dev/null || true; rm -rf "$WORK"' EXIT

# swarm-throughput parms, matching Tools/autotest/test_cluster_follow.py
printf 'AHRS_EKF_TYPE 10\nLOG_BACKEND_TYPE 0\nTERRAIN_ENABLE 0\nSCHED_LOOP_RATE 200\nEK3_ENABLE 0\nEK2_ENABLE 0\nSR0_RAW_SENS 0\nSR0_EXT_STAT 1\nSR0_RC_CHAN 0\nSR0_RAW_CTRL 0\nSR0_POSITION 1\nSR0_EXTRA1 0\nSR0_EXTRA2 0\nSR0_EXTRA3 0\nSR0_ADSB 0\nSIM_FLOAT_EXCEPT 0\nSIM_TEMP_BFACTOR 0\nSIM_TEMP_TCONST 1\nSIM_BATT_RES_OHM 0\nSRTL_POINTS 0\n' \
    > "$WORK/fast.parm"

export SITL_ADAPTIVE_RATE=1
export SITL_DISABLE_STACK_NANF=1
# SITL_HARD_NONBLOCK is deliberately NOT set here: it makes worker
# threads spin-yield instead of sleeping, which helps on a machine with
# cores to spare but strangles a 4-vCPU shared runner - three vehicles
# of spinning threads starve the physics threads they exist to serve

for i in $(seq 0 $((NVEHICLES-1))); do
    mkdir -p "$WORK/v$i"
    (cd "$WORK/v$i" && \
        exec "$COPTER" \
            --instance "$i" \
            --cluster $CLUSTER_ID \
            --model + \
            --speedup 100 \
            --wipe \
            --serial0 tcp:0 \
            --home -35.363261,149.165230,584,353 \
            --defaults "@ROMFS/default_params/copter.parm,$WORK/fast.parm" \
            > "$WORK/v$i.log" 2>&1) &
done

sleep 5

dump_logs() {
    echo "=== cluster-smoke: diagnostic dump ==="
    uname -a || true
    nproc || true
    netstat -an 2>/dev/null | grep -E "576[0-9]|577[0-9]|578[0-9]" || true
    for i in $(seq 0 $((NVEHICLES-1))); do
        echo "--- vehicle $i log"
        tail -60 "$WORK/v$i.log" 2>/dev/null || echo "(no log)"
    done
}

# drain every vehicle's SERIAL0 so nothing blocks on an unread
# outqueue; the first connect is given a long grace period because
# Windows Defender scans a freshly built unsigned exe on first launch
if ! python3 - "$RUN_SECONDS" $NVEHICLES <<'PY'
import socket
import sys
import time

run_seconds = float(sys.argv[1])
count = int(sys.argv[2])
socks = []
deadline = time.time() + run_seconds
for i in range(count):
    port = 5760 + 10 * i
    for _ in range(60):
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=5)
            s.setblocking(False)
            socks.append(s)
            print("cluster-smoke: connected to instance on port %u" % port)
            break
        except OSError:
            time.sleep(1)
    else:
        print("cluster-smoke: could not connect to port %u" % port)
        sys.exit(1)
while time.time() < deadline:
    time.sleep(0.2)
    for s in socks:
        try:
            while s.recv(65536):
                pass
        except OSError:
            pass
PY
then
    dump_logs
    exit 1
fi

kill $(jobs -p) 2>/dev/null || true
wait 2>/dev/null || true

echo "=== cluster-smoke: per-vehicle results ==="
FAIL=0
for i in $(seq 0 $((NVEHICLES-1))); do
    LOG="$WORK/v$i.log"
    # each vehicle must reach lock-step; the membership count it last
    # printed depends on join order (an early joiner may never re-print
    # after the final member arrives), so the full count is asserted
    # cluster-wide below rather than per vehicle
    if ! grep -q "in lock-step with" "$LOG"; then
        echo "vehicle $i: FAIL - never reached lock-step"
        FAIL=1
    fi
    if ! grep -q "adaptive rate" "$LOG"; then
        echo "vehicle $i: FAIL - governor made no rate decision"
        FAIL=1
    fi
    grep "adaptive rate" "$LOG" | tail -2 | sed "s/^/vehicle $i governor: /"
    grep "achieved" "$LOG" | grep -v "adaptive rate" | tail -3 | sed "s/^/vehicle $i: /"
done
if [ "$FAIL" != 0 ]; then
    dump_logs
    exit 1
fi
if ! grep -q "in lock-step with $NVEHICLES instances" "$WORK"/v*.log; then
    echo "cluster-smoke: FAIL - no vehicle ever saw all $NVEHICLES members"
    dump_logs
    exit 1
fi
echo "=== cluster-smoke: PASS - $NVEHICLES vehicles in lock-step, governor active ==="

#!/usr/bin/env bash
# Launch several SITL instances into one cluster and confirm they reach
# shared-memory lock-step - not merely that the processes stay alive.
# AP_FLAKE8_CLEAN (N/A - shell script)
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BINARY="${1:-${ROOT}/build/sitl/bin/arducopter}"
COUNT="${SITL_TEST_COUNT:-5}"
RUNTIME="${SITL_TEST_RUNTIME:-20}"

# Use a cluster id derived from our pid so a leftover segment from an
# earlier fleet, or a developer's own SITL session, cannot affect the
# result. Cluster ids are 0-255.
CLUSTER=$(( ($$ % 200) + 50 ))

if [ ! -x "${BINARY}" ]; then
    echo "FAIL: no SITL binary at ${BINARY} (./waf configure --board sitl && ./waf copter)"
    exit 1
fi

# name must match the /tmp/sitl_multi_* glob the CI workflow archives
LOGDIR="$(mktemp -d /tmp/sitl_multi_XXXXXX)"

PIDS=()
CLIENTS=()
cleanup() {
    kill ${PIDS[@]+"${PIDS[@]}"} ${CLIENTS[@]+"${CLIENTS[@]}"} 2>/dev/null
    wait 2>/dev/null
    rm -rf "${LOGDIR}"
}
trap cleanup EXIT

last=$((COUNT - 1))
for i in $(seq 0 "${last}"); do
    "${BINARY}" --instance "${i}" --cluster="${CLUSTER}" --speedup 100 --wipe \
        --home "-35.363261,149.165230,584,353" --model "+" \
        > "${LOGDIR}/inst${i}.log" 2>&1 &
    PIDS+=($!)
done

# SITL blocks on "Waiting for connection" until something attaches to
# SERIAL0, and the physics loop (which drives the barrier) never runs
# until then - so give every instance a client that stays connected and
# drains what it sends.
sleep 3
python3 -c '
import socket, sys, select
socks = []
for i in range(int(sys.argv[1])):
    s = socket.create_connection(("127.0.0.1", 5760 + 10 * i), timeout=10)
    s.setblocking(False)
    socks.append(s)
while True:
    r, _, _ = select.select(socks, [], [], 1.0)
    for s in r:
        try:
            s.recv(65536)
        except OSError:
            pass
' "${COUNT}" &
CLIENTS+=($!)

sleep "${RUNTIME}"

rc=0
for i in $(seq 0 "${last}"); do
    log="${LOGDIR}/inst${i}.log"
    if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
        echo "FAIL: instance ${i} exited early"
        rc=1
        continue
    fi
    if ! grep -q "in lock-step with" "${log}"; then
        echo "FAIL: instance ${i} never reached lock-step"
        grep "SharedMem" "${log}" | tail -5 | sed 's/^/    /'
        rc=1
    fi
    for bad in "bad magic" "sync timeout" "never initialised" "unusable" \
               "lock failed" "consistent failed"; do
        if grep -q "${bad}" "${log}"; then
            echo "FAIL: instance ${i} reported '${bad}'"
            grep "${bad}" "${log}" | tail -3 | sed 's/^/    /'
            rc=1
        fi
    done
done

# a single instance must NOT wait for anyone: --cluster on its own has no
# peers, and without --cluster the shared memory is never touched at all.
solo_log="${LOGDIR}/solo.log"
"${BINARY}" --instance 0 --speedup 100 --wipe \
    --home "-35.363261,149.165230,584,353" --model "+" > "${solo_log}" 2>&1 &
solo_pid=$!
sleep 3
kill "${solo_pid}" 2>/dev/null
if grep -q "SharedMem" "${solo_log}"; then
    echo "FAIL: run without --cluster touched shared memory"
    grep "SharedMem" "${solo_log}" | head -3 | sed 's/^/    /'
    rc=1
fi

if [ "${rc}" -eq 0 ]; then
    echo "PASS: ${COUNT} instances in lock-step in cluster ${CLUSTER}"
else
    echo "logs kept in ${LOGDIR}"
    trap - EXIT
    kill ${PIDS[@]+"${PIDS[@]}"} ${CLIENTS[@]+"${CLIENTS[@]}"} 2>/dev/null
fi
exit "${rc}"

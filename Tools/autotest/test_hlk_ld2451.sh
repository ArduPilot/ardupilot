#!/usr/bin/env bash
# test_hlk_ld2451.sh — Integration test for the HLK-LD2451 radar driver in ArduPilot SITL.
#
# What this script does:
#   1. Kills any leftover SITL / simulator processes on the relevant ports
#   2. Starts ArduCopter SITL with PRX1_TYPE=19 (HLK-LD2451), SERIAL2 configured
#      for the sensor protocol
#   3. Waits for SITL to initialise
#   4. Starts hlk_ld2451_sim.py in the background to feed frames on port 5763
#   5. Connects via pymavlink on port 5760, waits 15 s for the sensor to come up,
#      then checks:
#        a) PRX1_TYPE parameter reads back as 19
#        b) SYS_STATUS proximity-present bit (MAV_SYS_STATUS_SENSOR_PROXIMITY) is set
#   6. Reports PASS or FAIL and cleans up all background processes.
#
# Usage:
#   cd /path/to/ardupilot
#   Tools/autotest/test_hlk_ld2451.sh
#
# Requirements:
#   - ArduCopter SITL binary at build/sitl/bin/arducopter
#   - Python 3 with pymavlink (modules/mavlink/pymavlink)
#   - hlk_ld2451_sim.py in the same directory as this script

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

SITL_BIN="${REPO_ROOT}/build/sitl/bin/arducopter"
SIM_PY="${SCRIPT_DIR}/hlk_ld2451_sim.py"
DEFAULT_PARAMS="${SCRIPT_DIR}/default_params/copter.parm"

SITL_HOME="-35.363261,149.165230,584,353"
SITL_MODEL="+"
MAVLINK_PORT=5760
RADAR_PORT=5765
SITL_WAIT_S=10
SENSOR_WAIT_S=20

# ── Colour helpers ──────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${YELLOW}[TEST]${NC} $*"; }
pass()  { echo -e "${GREEN}[PASS]${NC} $*"; }
fail()  { echo -e "${RED}[FAIL]${NC} $*"; }

# ── Cleanup on exit ─────────────────────────────────────────────────────────
PIDS=()
cleanup() {
    info "Cleaning up background processes: ${PIDS[*]:-none}"
    for pid in "${PIDS[@]}"; do
        kill "${pid}" 2>/dev/null || true
    done
    # Belt-and-suspenders: kill any still-running SITL / sim processes that
    # use our ports
    fuser -k "${MAVLINK_PORT}/tcp" 2>/dev/null || true
    fuser -k "${RADAR_PORT}/tcp"   2>/dev/null || true
    rm -f /tmp/hlk_ld2451_test_params.parm
}
trap cleanup EXIT

# ── Sanity checks ───────────────────────────────────────────────────────────
if [ ! -x "${SITL_BIN}" ]; then
    fail "ArduCopter SITL binary not found: ${SITL_BIN}"
    fail "Build it with: ./waf configure --board sitl && ./waf build"
    exit 1
fi

if [ ! -f "${SIM_PY}" ]; then
    fail "Simulator script not found: ${SIM_PY}"
    exit 1
fi

if [ ! -f "${DEFAULT_PARAMS}" ]; then
    fail "Default params not found: ${DEFAULT_PARAMS}"
    exit 1
fi

# ── Kill anything already occupying the ports ───────────────────────────────
info "Freeing ports ${MAVLINK_PORT} and ${RADAR_PORT} ..."
fuser -k "${MAVLINK_PORT}/tcp" 2>/dev/null || true
fuser -k "${RADAR_PORT}/tcp"   2>/dev/null || true
sleep 1

# ── Build a temp param file that adds our radar config ──────────────────────
PARAM_FILE=/tmp/hlk_ld2451_test_params.parm
cat > "${PARAM_FILE}" << 'EOF'
PRX1_TYPE       19
SERIAL4_PROTOCOL 11
SERIAL4_BAUD    115
EOF
info "Extra params written to ${PARAM_FILE}"

# ── Start ArduCopter SITL ───────────────────────────────────────────────────
info "Starting ArduCopter SITL ..."
"${SITL_BIN}" \
    --home "${SITL_HOME}" \
    --model "${SITL_MODEL}" \
    --defaults "${DEFAULT_PARAMS},${PARAM_FILE}" \
    --serial4 tcp:"${RADAR_PORT}" \
    -w \
    > /tmp/sitl_stdout.log 2>&1 &
SITL_PID=$!
PIDS+=("${SITL_PID}")
info "SITL PID=${SITL_PID}"

info "Waiting ${SITL_WAIT_S}s for SITL to initialise ..."
sleep "${SITL_WAIT_S}"

# Verify SITL is still running
if ! kill -0 "${SITL_PID}" 2>/dev/null; then
    fail "SITL exited prematurely.  Last lines of log:"
    tail -20 /tmp/sitl_stdout.log || true
    exit 1
fi

# ── Start the radar simulator ────────────────────────────────────────────────
info "Starting HLK-LD2451 simulator on port ${RADAR_PORT} ..."
python3 "${SIM_PY}" \
    --host 127.0.0.1 \
    --port "${RADAR_PORT}" \
    --scenario approach \
    > /tmp/radar_sim_stdout.log 2>&1 &
SIM_PID=$!
PIDS+=("${SIM_PID}")
info "Radar sim PID=${SIM_PID}"

# ── Run pymavlink checks ─────────────────────────────────────────────────────
info "Running pymavlink checks (${SENSOR_WAIT_S}s timeout) ..."

python3 - <<PYEOF
import sys
import time

sys.path.insert(0, '${REPO_ROOT}/modules/mavlink/pymavlink')

from pymavlink import mavutil

MAV_SYS_STATUS_SENSOR_PROXIMITY = (1 << 20)  # MAV_SYS_STATUS_OBSTACLE_AVOIDANCE

MAVLINK_PORT = int(${MAVLINK_PORT})
SENSOR_WAIT  = int(${SENSOR_WAIT_S})

print(f"[CHECK] Connecting to SITL at tcp:127.0.0.1:{MAVLINK_PORT} ...")
try:
    mav = mavutil.mavlink_connection(
        f"tcp:127.0.0.1:{MAVLINK_PORT}",
        timeout=10,
    )
    mav.wait_heartbeat(timeout=15)
except Exception as exc:
    print(f"[CHECK] Could not connect to SITL: {exc}", file=sys.stderr)
    sys.exit(2)

print(f"[CHECK] Heartbeat received from system {mav.target_system} component {mav.target_component}")

# Request data streams so SYS_STATUS is sent
mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
    0,  # MAV_DATA_STREAM_ALL
    5, 1)

# ── Check 1: PRX1_TYPE parameter ────────────────────────────────────────────
print("[CHECK] Requesting PRX1_TYPE parameter ...")
mav.mav.param_request_read_send(
    mav.target_system,
    mav.target_component,
    b"PRX1_TYPE",
    -1,
)
deadline = time.monotonic() + 10
prx_type = None
while time.monotonic() < deadline:
    msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
    if msg and msg.param_id.rstrip("\x00") == "PRX1_TYPE":
        prx_type = int(msg.param_value)
        break

if prx_type != 19:
    print(f"[CHECK] FAIL: PRX1_TYPE={prx_type} (expected 19)", file=sys.stderr)
    sys.exit(1)
print(f"[CHECK] PRX1_TYPE = {prx_type}  OK")

# ── Check 2: SYS_STATUS proximity present bit ────────────────────────────────
print(f"[CHECK] Waiting up to {SENSOR_WAIT}s for proximity sensor to appear in SYS_STATUS ...")
deadline = time.monotonic() + SENSOR_WAIT
prox_present = False
while time.monotonic() < deadline:
    msg = mav.recv_match(type="SYS_STATUS", blocking=True, timeout=1)
    if msg is None:
        continue
    if msg.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_PROXIMITY:
        prox_present = True
        print(
            f"[CHECK] Proximity sensor present=1 enabled={bool(msg.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_PROXIMITY)}"
            f" health={bool(msg.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_PROXIMITY)}"
        )
        break

if not prox_present:
    print("[CHECK] FAIL: SYS_STATUS proximity present bit never set", file=sys.stderr)
    sys.exit(1)
print("[CHECK] SYS_STATUS proximity present bit is set  OK")

sys.exit(0)
PYEOF

CHECK_EXIT=$?

# ── Report result ────────────────────────────────────────────────────────────
if [ "${CHECK_EXIT}" -eq 0 ]; then
    pass "HLK-LD2451 integration test PASSED"
    info "SITL log (last 10 lines):"
    tail -10 /tmp/sitl_stdout.log || true
else
    fail "HLK-LD2451 integration test FAILED (exit code ${CHECK_EXIT})"
    info "SITL log tail:"
    tail -30 /tmp/sitl_stdout.log || true
    info "Radar sim log tail:"
    tail -20 /tmp/radar_sim_stdout.log || true
    exit 1
fi

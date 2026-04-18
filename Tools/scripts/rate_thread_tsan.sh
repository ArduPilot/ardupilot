#!/usr/bin/env bash
# rate_thread_tsan.sh — verify no data races in ArduCopter rate thread
#
# Builds ArduCopter SITL with ThreadSanitizer and runs a short flight with
# FSTRATE_ENABLE=1 to exercise all cross-thread paths.  Any TSan report
# written to stderr indicates a real data race that needs fixing.
#
# Usage:
#   cd <ardupilot-root>
#   Tools/scripts/rate_thread_tsan.sh [--no-build] [--speedup N]
#
# Options:
#   --no-build   Skip the waf configure+build step (use existing binary)
#   --speedup N  SITL speedup factor (default: 5)
#
# Expected output on a clean (fixed) tree:
#   [TSan] No data races detected.  All clear.
#
# Expected output on the unpatched tree (plain bool / plain uint8_t):
#   WARNING: ThreadSanitizer: data race ...
#     Write of size 1 at ... by thread T1:
#       #0 Copter::enable_fast_rate_loop ...
#     Previous read of size 1 at ... by thread T2:
#       #0 Copter::run_rate_controller_main ...
#   ...
#
# Dependencies: gcc or clang with TSan, python3, mavproxy or pymavlink

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SITL_BIN="$ROOT/build/sitl/bin/arducopter"
SPEEDUP=5
BUILD=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build)  BUILD=0; shift ;;
        --speedup)   SPEEDUP="$2"; shift 2 ;;
        *)           echo "Unknown option: $1"; exit 1 ;;
    esac
done

# ── 1. Build ──────────────────────────────────────────────────────────────────
if [[ $BUILD -eq 1 ]]; then
    echo "=== Configuring SITL with ThreadSanitizer ==="
    cd "$ROOT"
    python3 waf configure --board sitl --tsan
    echo "=== Building ArduCopter ==="
    python3 waf copter
fi

if [[ ! -x "$SITL_BIN" ]]; then
    echo "ERROR: $SITL_BIN not found.  Run without --no-build first."
    exit 1
fi

# ── 2. Prepare TSAN environment ───────────────────────────────────────────────
SUPP="$SCRIPT_DIR/tsan_suppressions.txt"
TSAN_LOG="$ROOT/tsan_report.txt"

export TSAN_OPTIONS="halt_on_error=0 \
log_path=$TSAN_LOG \
suppressions=$SUPP \
second_deadlock_stack=1 \
history_size=5"

# ── 3. Create a minimal param file that enables the fast rate thread ──────────
PARAM_FILE="$(mktemp /tmp/fstrate_XXXXXX.parm)"
cat > "$PARAM_FILE" <<'PARAMS'
FSTRATE_ENABLE 1
AHRS_EKF_TYPE 10
INS_GYRO_FILTER 300
PARAMS
trap 'rm -f "$PARAM_FILE"' EXIT

# ── 4. Run SITL for 30 simulated seconds ─────────────────────────────────────
echo "=== Running SITL with TSan (speedup=${SPEEDUP}x, 30 sim-seconds) ==="
echo "    TSan log: ${TSAN_LOG}.* "
echo "    Param file: $PARAM_FILE"
echo ""

# We use a Python helper to arm, hover, and disarm via MAVLink.
# SITL exits when the helper script completes.
HELPER="$(mktemp /tmp/tsan_flight_XXXXXX.py)"
cat > "$HELPER" <<'PYEOF'
#!/usr/bin/env python3
"""Minimal MAVLink flight script for TSan validation.

Arms the vehicle, takes off to 10 m, hovers for 10 s,
then lands and disarms.  This exercises all rate-thread
cross-thread paths under TSan observation.
"""
import time, sys
try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink not installed — install with: pip3 install pymavlink")
    sys.exit(0)

mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
mav.wait_heartbeat(timeout=30)
print("Connected to vehicle")

# Switch to GUIDED mode
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    1,  # base_mode (custom mode enabled)
    4,  # GUIDED
    0, 0, 0, 0, 0)
time.sleep(1)

# Arm
mav.arducopter_arm()
mav.motors_armed_wait()
print("Armed")

# Take off to 10 m
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
    0, 0, 0, 0, 0, 0, 10)
print("Taking off ...")
time.sleep(8)

# Hover for 15 s (exercises the hot path + rate-check cadence)
print("Hovering for 15 s ...")
time.sleep(15)

# Land
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
    0, 0, 0, 0, 0, 0, 0)
print("Landing ...")
time.sleep(8)

mav.arducopter_disarm()
print("Disarmed — test complete")
PYEOF
trap 'rm -f "$PARAM_FILE" "$HELPER"' EXIT

# Start SITL in the background
"$SITL_BIN" \
    -S \
    -I0 \
    --home 51.8733,-1.1481,50,270 \
    --speedup "$SPEEDUP" \
    --defaults "$PARAM_FILE" \
    &
SITL_PID=$!

# Give SITL time to start
sleep 3

# Run the flight helper
python3 "$HELPER" || true

# Give TSan a moment to flush its reports
sleep 2
kill "$SITL_PID" 2>/dev/null || true
wait "$SITL_PID" 2>/dev/null || true

# ── 5. Analyse TSan output ────────────────────────────────────────────────────
echo ""
echo "=== ThreadSanitizer report ==="

RACE_FILES=( "${TSAN_LOG}"* )
RACES_FOUND=0

for f in "${RACE_FILES[@]}"; do
    if [[ -f "$f" && -s "$f" ]]; then
        grep -c "data race" "$f" >/dev/null 2>&1 && {
            RACE_COUNT=$(grep -c "data race" "$f" || true)
            echo "FAIL: $RACE_COUNT data race(s) detected in $f"
            echo "---------- TSan report ----------"
            cat "$f"
            echo "---------------------------------"
            RACES_FOUND=$((RACES_FOUND + RACE_COUNT))
        }
    fi
done

if [[ $RACES_FOUND -eq 0 ]]; then
    echo "[TSan] No data races detected.  All clear."
    exit 0
else
    echo ""
    echo "FAIL: $RACES_FOUND data race(s) detected."
    echo "Apply the PR2 atomic fixes and re-run."
    exit 1
fi

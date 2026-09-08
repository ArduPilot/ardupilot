#!/bin/bash
# Resilient flash for any AP_HAL_Zephyr board. Use this instead of calling
# uploader.py directly - a bare upload fails often enough to cost measurements.
#
# Usage: Tools/scripts/zephyr_flash.sh [image.apj] [max_attempts]
#        default image: build/mr_vmu_rt1176/zephyr_upload.apj
#
# Board is derived from the .apj path; mr_vmu_rt1176, CubeOrangeZephyr and
# ESP32S3Zephyr are known. See the case statement below to add one.
#
# ── FOUR RULES YOU CANNOT BREAK ───────────────────────────────────────────────
#
# 1. NO --port. uploader.py does its own /dev/serial/by-id globbing and must
#    catch the bootloader name when the board re-enumerates mid-flash. Pinning
#    --port is only safe as a GLOB covering BOTH identities, and on some boards
#    they do not share a prefix - CubeOrange is 'ArduPilot_CubeOrange_' in the
#    app but 'Hex_ProfiCNC_CubeOrange-BL_' in the bootloader. A glob matching
#    one and not the other leaves uploader.py with nothing to send its reboot
#    to, which looks EXACTLY like a board that will not enter the bootloader.
#    (If a shared bench forces pinning, pass both globs comma-separated.)
#
# 2. A HARDWARE reset must be fired ~3 s in, INTO uploader's retry loop, when
#    the board is sitting in its APP. The default soft reset traps the RT1176
#    in BootROM at 0x223104; only a real reset gets it out.
#
# 3. DO NOT reset when the board is already in the bootloader. A '-BL' in the
#    by-id name means the bootloader is up and WAITING for uploader.py - a
#    reset there restarts the board mid-handshake and destroys the window you
#    already had. hw_reset is gated on `bl_present` for this reason.
#
# 4. The reset must match the attached probe. pyocd only sees CMSIS-DAP
#    probes; on a Black Magic Probe bench `pyocd list` reports NO probes and
#    the reset silently does nothing. hw_reset() picks the right one.
#
# ── FAILURE MODES THIS HANDLES, all observed on hardware ──────────────────────
#
#   "bootloader reports INVALID OPERATION" partway through Program
#       AP_Bootloader (ours, PX4-protocol-compatible) rejecting a write. Seen
#       at 6-12% twice in a row, then succeeding after a hardware reset, so it
#       is a state/sequencing fault rather than a bad image. Retry + reset
#       clears it. Root cause is still open in
#       Tools/AP_Bootloader/support_Zephyr.cpp's ROM-API path.
#
#   uploader exits 124 (timeout)
#       Board never entered the bootloader. Reset and retry.
#
#   Board left sitting in the bootloader ('-BL' in the by-id name)
#       Flash is half-written. uploader.py can talk to it directly, so just
#       retry - do NOT power cycle first, that loses the bootloader session.
#
#   Board wedged with no app and no bootloader
#       Escalating hardware resets. If every attempt fails, a PHYSICAL power
#       cycle of BOTH probe and board together is the documented last resort -
#       a soft reset does not clear it.
#
#   Orphaned uploader.py from an earlier interrupted run
#       It never exits, keeps re-globbing, and CONSUMES the bootloader window
#       this run needs - while printing the same "waiting for the bootloader"
#       line, so the log looks identical to a board that will not reset.
#       Killed at the top of this script.
#
# Verifies afterwards that the board re-enumerates WITHOUT the bootloader
# suffix, i.e. the app is actually running. A "successful" upload that leaves
# the board in the bootloader is a failure for our purposes and reported as one.
set -u

APJ=${1:-build/mr_vmu_rt1176/zephyr_upload.apj}
MAX=${2:-5}

cd "$(dirname "$0")/../.." || exit 1

if [ ! -f "$APJ" ]; then
    echo "FLASH FAIL: $APJ does not exist"
    exit 1
fi

# ── Board identity + reset method ────────────────────────────────────────────
# Derived from the .apj path so one script serves every Zephyr board. Two
# things vary per board and both have bitten us:
#
#   1. The app and bootloader USB identities can have DIFFERENT vendor
#      prefixes. CubeOrange is 'ArduPilot_CubeOrange_' in the app and
#      'Hex_ProfiCNC_CubeOrange-BL_' in the bootloader, so a single glob that
#      matches one misses the other - which looks exactly like a board that
#      will not enter the bootloader.
#   2. pyocd only sees CMSIS-DAP probes. On a Black Magic Probe bench
#      `pyocd list` reports nothing, so the reset silently does nothing.
case "$APJ" in
    *CubeOrangeZephyr*) APP_PAT='ArduPilot_CubeOrange_'; BL_PAT='CubeOrange-BL_' ;;
    *ESP32S3Zephyr*)    APP_PAT='ESP32S3'; BL_PAT='ESP32S3.*-BL_' ;;
    *)                  APP_PAT='ArduPilot.*RT1176_'; BL_PAT='ArduPilot.*RT1176-BL_' ;;
esac

app_present() { ls /dev/serial/by-id/ 2>/dev/null | grep -q "$APP_PAT"; }
bl_present()  { ls /dev/serial/by-id/ 2>/dev/null | grep -q "$BL_PAT"; }

# Hardware reset, whichever probe is actually attached.
hw_reset() {
    if python3 -m pyocd list 2>/dev/null | grep -qv 'No available'; then
        timeout 30 pyocd reset -m hw >/dev/null 2>&1
    else
        timeout 40 python3 Tools/scripts/zephyr_pin_reset_bmp.py >/dev/null 2>&1
    fi
}

# Kill orphaned uploaders: one left over from an interrupted run steals the
# bootloader window this run needs. By PID - `pkill -f uploader` would match
# this script's own command line.
ps -eo pid,args --no-headers | awk '/uploader\.py/ && !/awk/ {print $1}' |
    while read -r p; do kill -9 "$p" 2>/dev/null; done

for attempt in $(seq 1 "$MAX"); do
    echo "== flash attempt $attempt/$MAX: $APJ"

    timeout 150 python3 Tools/scripts/uploader.py "$APJ" > /tmp/zephyr_flash_$$.log 2>&1 &
    UP=$!
    sleep 3
    hw_reset
    wait $UP
    rc=$?

    if [ $rc -eq 0 ]; then
        # Uploader is happy; confirm the APP is what came back, not the bootloader.
        sleep 5
        if app_present && ! bl_present; then
            echo "== flash OK on attempt $attempt"
            rm -f /tmp/zephyr_flash_$$.log
            exit 0
        fi
        echo "   uploader reported success but the board is not running the app"
    else
        # -a: uploader.py's log contains raw protocol bytes, so grep treats it as
        # binary and prints "binary file matches" INSTEAD of the match - which
        # made every failure report an empty reason.
        echo "   uploader rc=$rc: $(grep -aoE 'ERROR.*|INVALID OPERATION|Timeout.*' /tmp/zephyr_flash_$$.log | tail -1)"
    fi

    # Escalate: hardware reset, and give the board longer each time. If it is
    # sitting in the bootloader that is FINE - uploader.py talks to it there,
    # so do NOT reset in that state, it would restart the board mid-handshake.
    bl_present || hw_reset
    sleep $((attempt * 3))
done

echo "FLASH FAIL after $MAX attempts."
echo "  Board state: app=$(app_present && echo yes || echo no) bootloader=$(bl_present && echo yes || echo no)"
echo "  Last log:"
tail -5 /tmp/zephyr_flash_$$.log 2>/dev/null | sed 's/^/    /'
echo "  Next step is a PHYSICAL power cycle of the probe AND the board together;"
echo "  a soft reset does not clear this state and needs a human."
rm -f /tmp/zephyr_flash_$$.log
exit 1

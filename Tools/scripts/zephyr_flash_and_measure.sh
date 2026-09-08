#!/bin/bash
# Flash mr_vmu_rt1176 and take a non-intrusive measurement, unattended.
#
# BOTH HALVES OF THE FLASH RECIPE ARE REQUIRED - neither works alone, and it
# took a session to find:
#   * bare uploader.py with NO --port. uploader.py does its own by-id globbing
#     and must catch the bootloader's `-BL` name when the board re-enumerates
#     mid-flash. Passing --port replaces that glob and the upload dies.
#   * `pyocd reset -m hw` fired ~3 s later, INTO uploader's retry loop. The
#     default soft reset traps the core in BootROM at 0x223104, so -m hw
#     (hardware reset) is not optional.
#
# Usage: Tools/scripts/zephyr_flash_and_measure.sh [boot_wait_s] [sample_s]
set -u

BOARD=mr_vmu_rt1176
APJ="build/${BOARD}/zephyr_upload.apj"
ELF="build/${BOARD}/zephyr_build/zephyr/zephyr.elf"
BOOT_WAIT=${1:-45}
SAMPLE=${2:-15}

cd "$(dirname "$0")/../.." || exit 1

if [ ! -f "$APJ" ]; then
    echo "FAIL: $APJ missing - did the build succeed?"
    exit 1
fi
# A failed build leaves the LAST-GOOD apj sitting in place, which looks exactly
# like a successful one. This cost a bad flash once; check the artefact is not
# older than the ELF it should have come from.
if [ "$APJ" -ot "$ELF" ]; then
    echo "FAIL: $APJ is OLDER than $ELF - stale artefact, refusing to flash."
    exit 1
fi
echo "== flashing $APJ ($(stat -c %y "$APJ"))"

# Delegate to the resilient flasher - a bare uploader.py call fails often
# enough to lose measurements. It retries, fires the hardware reset into
# uploader's retry loop, and verifies the APP (not the bootloader) came back.
Tools/scripts/zephyr_flash.sh "$APJ" || {
    echo "FAIL: could not flash. Not measuring a board whose firmware state"
    echo "      is unknown."
    exit 1
}

# DETECT readiness, do not guess at it. A fixed sleep is wrong both ways: it
# wastes time when boot is quick and still samples a booting board when it is
# not. Sampling mid-boot produced "3.9 Hz" and "16.5 Hz" readings that were
# reported as loop rates. zephyr_wait_ready.py polls g_ap_prof over SWD until
# the bus-callback rate plateaus and the loop rate stops changing.
echo "== waiting for steady state (polled, max ${BOOT_WAIT}s)"
Tools/scripts/zephyr_wait_ready.py "$ELF" "$BOOT_WAIT" || {
    echo "FAIL: board never reached steady state - not measuring it."
    exit 1
}

# Evidence goes in the repo, not /tmp - these captures are proof-of-work for a
# paid HAL contract and are what any conclusion here rests on. STABLE filenames,
# so git shows the diff from run to run; the history is the archive.
echo "== sampling ${SAMPLE}s"
python3 Tools/scripts/zephyr_xfer_hist.py "$SAMPLE" --elf "$ELF" 2>&1 | tee profile.txt
echo "== profile evidence written to ./profile.txt"

# threads.txt + tasks.txt, rendered on-target into g_ap_sysinfo because MAVFTP
# cannot serve them on this board. Needs --enable-stats for the CPU LOAD% column.
echo "== reading @SYS/threads.txt + @SYS/tasks.txt over SWD"
python3 Tools/scripts/zephyr_sysinfo.py --elf "$ELF" --wait 6 || \
    echo "   (sysinfo unavailable - needs the g_ap_sysinfo build)"

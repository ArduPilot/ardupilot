#!/bin/bash
# zephyr_build_progress.sh <logfile> <target_pct>
#
# Fire once when a waf build passes <target_pct>, so a long build reports 25/50/75
# instead of going silent for minutes.
#
# THE LOG MUST NOT BE PIPED THROUGH tail/head. waf prints "[n/m] task" lines as it
# goes, but tail cannot emit anything until its input closes, so a log captured as
#     ./waf copter | tail -40 > build.log
# stays EMPTY until the build finishes and this watcher sees nothing to report.
# Capture the whole thing instead and tail the FILE afterwards:
#     ./waf copter > build.log 2>&1
#
# Exits 0 when the milestone is reached, or when the build ends first.
set -u
LOG="${1:?usage: zephyr_build_progress.sh <logfile> <target_pct>}"
TARGET="${2:-25}"
POLL="${POLL:-3}"
DEADLINE=$(( $(date +%s) + ${MAX_WAIT:-3600} ))

while [ "$(date +%s)" -lt "$DEADLINE" ]; do
    if [ -f "$LOG" ]; then
        # Build finished (or failed) before this milestone: say so and stop, so
        # a chain of watchers cannot outlive the build it was watching.
        # Only the VEHICLE build ending counts. "'configure' finished
        # successfully" also matches a bare "finished successfully" and made
        # this fire instantly on any log that had configure output in it.
        if grep -qE "'(copter|plane|rover|sub|heli|antennatracker|bootloader|examples|tests|check)' finished successfully|Build failed|^[A-Z_]*EXIT=" "$LOG" 2>/dev/null; then
            echo "BUILD-ENDED before ${TARGET}% - $(grep -cE '^\[[0-9]+/[0-9]+\]' "$LOG" 2>/dev/null) task lines seen"
            exit 0
        fi
        # waf numbers its own tasks, then the Zephyr/ninja link starts again
        # at [1/5]. Take the LARGEST total seen and the latest count against
        # THAT total, so the short ninja sequence cannot reset the percentage
        # to near zero - a fixed "m > 100" floor did that job but also ignored
        # every genuinely small build.
        BIGM=$(grep -oE '^\[[0-9]+/[0-9]+\]' "$LOG" 2>/dev/null \
               | tr -d '[]' | cut -d/ -f2 | sort -n | tail -1)
        if [ -n "${BIGM:-}" ]; then
            N=$(grep -oE "^\[[0-9]+/${BIGM}\]" "$LOG" 2>/dev/null \
                | tail -1 | tr -d '[]' | cut -d/ -f1)
            M=$BIGM
            if [ -n "${N:-}" ]; then
                PCT=$(( N * 100 / M ))
                if [ "$PCT" -ge "$TARGET" ]; then
                    echo "BUILD ${PCT}% (${N}/${M}) - milestone ${TARGET}%"
                    exit 0
                fi
            fi
        fi
    fi
    sleep "$POLL"
done
echo "WATCH-TIMEOUT waiting for ${TARGET}%"
exit 1

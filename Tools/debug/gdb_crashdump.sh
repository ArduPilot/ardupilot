#!/usr/bin/env bash

# script to more easily get a backtrace from an ArduPilot crash_dump.bin

[ $# -eq 2 ] || {
    echo "Usage: gdb_crashdump.sh ELF_FILE CRASH_DUMP"
    exit 1
}

ELF_FILE="$1"
CRASH_DUMP="$2"
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CHIBIOS_GDB_COMMANDS="$SCRIPT_DIR/chibios_crash.gdb"
CRASHDEBUG="$SCRIPT_DIR/debug_interface.py"

# if the dump file has 0xFF padding (from SD card pre-allocated crash dump),
# truncate to the actual dump size stored in the last 4 bytes of the last sector
CRASH_DUMP_FIXED="$CRASH_DUMP"
cleanup() {
    if [ "$CRASH_DUMP_FIXED" != "$CRASH_DUMP" ]; then
        rm -f "$CRASH_DUMP_FIXED"
    fi
}
trap cleanup EXIT

if ! DUMP_SIZE=$(python3 "$SCRIPT_DIR/crashdump_info.py" "$ELF_FILE" "$CRASH_DUMP"); then
    exit 1
fi

if [ -n "$DUMP_SIZE" ]; then
    CRASH_DUMP_FIXED=$(mktemp /tmp/crashdump_XXXXXX.bin)
    head -c "$DUMP_SIZE" "$CRASH_DUMP" > "$CRASH_DUMP_FIXED"
    echo "Truncated dump from $(stat -c%s "$CRASH_DUMP") to $DUMP_SIZE bytes"
fi

printf -v ELF_FILE_QUOTED '%q' "$ELF_FILE"
printf -v CRASH_DUMP_QUOTED '%q' "$CRASH_DUMP_FIXED"
printf -v CRASHDEBUG_QUOTED '%q' "$CRASHDEBUG"
GDB_TARGET_COMMAND="target remote | python3 $CRASHDEBUG_QUOTED --elf $ELF_FILE_QUOTED --dump $CRASH_DUMP_QUOTED"
echo "ChibiOS thread support loaded; use 'info threads', 'thread N' or 'thread apply all bt' in GDB"
arm-none-eabi-gdb -nx "$ELF_FILE" \
    -x "$CHIBIOS_GDB_COMMANDS" \
    -ex "set target-charset ASCII" \
    -ex "$GDB_TARGET_COMMAND" \
    -ex "info threads"

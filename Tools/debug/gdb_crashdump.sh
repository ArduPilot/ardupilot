#!/bin/bash

# script to more easily get a backtrace from an ArduPilot crash_dump.bin

[ $# -eq 2 ] || {
    echo "Usage: gdb_crashdump.sh ELF_FILE CRASH_DUMP"
    exit 1
}

ELF_FILE=$1
CRASH_DUMP=$2

arm-none-eabi-gdb -nx "$ELF_FILE" -ex "set target-charset ASCII" -ex "target remote | modules/CrashDebug/bins/lin64/CrashDebug --elf $ELF_FILE --dump $CRASH_DUMP"

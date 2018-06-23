#!/bin/sh
# make an intel hex file including bootloader, for loading with DFU

if [ $# -lt 4 ]; then
    echo "Usage: make_intel_hex.sh BINFILE BOOTLOADER RESERVE_KB HEXFILEOUT"
    exit 1
fi

SCRIPTS=$(dirname $0)

BINFILE="$1"
BOOTLOADERFILE="$2"
RESERVE_KB="$3"
HEXFILE="$4"

[ -f "$BINFILE" ] || {
    echo "Can't find bin file $BINFILE"
    exit 1
}

[ -f "$BOOTLOADERFILE" ] || {
    echo "Can't find bootloader file $BOOTLOADERFILE"
    exit 1
}

cat "$BOOTLOADERFILE" > "$HEXFILE".tmp
dd bs=1024 seek=$RESERVE_KB if="$BINFILE" of="$HEXFILE".tmp 2>&1
"$SCRIPTS"/bin2hex.py --offset 0x08000000 "$HEXFILE".tmp "$HEXFILE"
rm -f "$HEXFILE".tmp
echo "Created $HEXFILE"

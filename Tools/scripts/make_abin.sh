#!/bin/sh
# make an abin file for a firmware this file format is for sending to
# a memory constrained companion computer to flash over serial to a
# flight board

if [ $# -lt 3 ]; then
    echo "Usage: make_abin.sh BINFILE ABINFILE GITHASH"
    exit 1
fi

BINFILE="$1"
ABINFILE="$2"
githash="$3"

[ -f "$BINFILE" ] || {
    echo "Can't find bin file $BINFILE for abin"
    exit 1
}

[ -n "$githash" ] || {
    echo "Empty git hash for abin"
    exit 1
}

sum=$(md5sum "$BINFILE" | cut -d' ' -f1)

echo "githash $githash md5 $sum"

cat <<EOF > "$ABINFILE"
git version: $githash
MD5: $sum
--
EOF
cat "$BINFILE" >> "$ABINFILE"

echo "Created $ABINFILE"


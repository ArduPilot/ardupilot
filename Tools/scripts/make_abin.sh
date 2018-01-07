#!/bin/sh
# make an abin file for a firmware this file format is for sending to
# a memory constrained companion computer to flash over serial to a
# flight board

if [ $# -lt 1 ]; then
    # default to FMUv3
    ELF=build/fmuv3/bin/arducopter
else
    ELF=$1
fi

if [ $# -lt 2 ]; then
    VEHICLE="arducopter"
else
    VEHICLE="$2"
fi

[ -f $ELF ] || {
    echo "Can't find ELF file"
    exit 1
}

echo "Creating $VEHICLE.bin"
arm-none-eabi-objcopy -O binary "$ELF" "$VEHICLE".bin || {
    echo "Failed to create bin file"
    exit 1
}

sum=$(md5sum "$VEHICLE".bin | cut -d' ' -f1)
githash=$(git rev-parse HEAD)

echo "githash $githash md5 $sum"

cat <<EOF > "$VEHICLE".abin
git version: $githash
MD5: $sum
--
EOF
cat "$VEHICLE".bin >> "$VEHICLE".abin

echo "Created $VEHICLE.abin"

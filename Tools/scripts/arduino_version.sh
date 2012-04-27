#!/bin/sh
# find arduino version

ARDUINO=$1
if test -f $ARDUINO/lib/version.txt; then
    # arduino 1.0 uses this file
    ver=$(head -1 $ARDUINO/lib/version.txt | cut -c1-4)
    # cope with pre 1.0 versions
    leading=$(echo $ver | cut -c1)
    if [ "$leading" = "0" ]; then
	echo $ver
	exit 0
    fi
    major=$(echo $ver | cut -d. -f1)
    minor=$(echo $ver | cut -d. -f2)
    v=$(expr $major \* 100 + $minor)
    echo $v
elif test -f $ARDUINO/revisions.txt; then
    ARDUINO_VER=$(head -1 $ARDUINO/revisions.txt | cut -d' ' -f 2)
else
    echo "UNKNOWN"
fi

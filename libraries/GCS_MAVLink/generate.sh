#!/bin/sh
# script to re-generate mavlink C code for APM

mavdir="$(dirname $0)"
dname="$(basename $mavdir)"
[ "$dname" = "GCS_MAVLink" ] || {
    echo "This script should be run from the ardupilot directory with the command ./libraries/GCS_MAVLink/generate.sh"
    exit 1
}

if ! which mavgen.py > /dev/null; then
    echo "mavgen.py must be in your PATH. Get it from http://github.com/mavlink/mavlink in the pymavlink/generator directory"
    exit 1
fi

echo "Removing old includes"
rm -rf "$mavdir/include/*"

echo "Generating C code"
mavgen.py --lang=C --wire-protocol=1.0 --output=$mavdir/include/mavlink/v1.0 $mavdir/message_definitions/ardupilotmega.xml

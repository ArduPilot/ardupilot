#!/bin/bash

set -e
set -x

BOARD=NucleoH743
BOARD=MatekH743
#BOARD=F35Lightning

THISDIR=$(dirname $0)

VEHICLE="copter"
DEFAULTS_PATH=""
EXTRA_HWDEF=""

usage() {
    echo "Usage: $0 [-v <plane|copter>] [-d DEFAULT_FILEPATH] [-b BOARDNAME]" >&2;
    exit 1;
}

while getopts ":v:d:b:" o; do
    case "${o}" in
        v)
            VEHICLE=${OPTARG}
            [ "$VEHICLE" == "plane" ] || [ "$VEHICLE" == "copter" ] || usage
            ;;
        d)
            DEFAULTS_PATH=${OPTARG}
            ;;
        b)
            BOARD=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))

if [ -z "${DEFAULTS_PATH}" ]; then
    if [ "$VEHICLE" == "plane" ]; then
        DEFAULTS_PATH="$THISDIR/plane-default.param"
        EXTRA_HWDEF="$THISDIR/plane-extra-hwdef-sitl-on-hw.dat"
    elif ["$VEHICLE" == "copter" ]; then
        DEFAULTS_PATH="$THISDIR/default.param"
        EXTRA_HWDEF="$THISDIR/extra-hwdef-sitl-on-hw.dat"
    fi
fi

./waf configure \
      --board=$BOARD \
      --extra-hwdef="$EXTRA_HWDEF" \
      --default-param="$DEFAULTS_PATH"

./waf ${VEHICLE} --upload

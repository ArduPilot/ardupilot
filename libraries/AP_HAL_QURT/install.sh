#!/bin/bash
# script to install ArduPilot on a VOXL board
# this assumes you have already installed the voxl-ardupilot.service file
# and /usr/bin/voxl-ardupilot script

USE_ADB=false

usage() {
    echo "Usage: install.sh -a BOARD"
    echo "       install.sh BOARD IPADDRESS"
    echo ""
    echo "  BOARD      Board name (ModalAI-VOXL2 or ModalAI-VOXL3)"
    echo "  IPADDRESS  IP address of target (required when using rsync)"
    echo "  -a         Use adb push instead of rsync"
    exit 1
}

while getopts "a" opt; do
    case $opt in
        a)
            USE_ADB=true
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND - 1))

# Board is the first positional argument (required)
[ $# -ge 1 ] || usage
BOARD="$1"
shift

if [ "$BOARD" != "ModalAI-VOXL2" ] && [ "$BOARD" != "ModalAI-VOXL3" ]; then
    echo "Error: BOARD must be ModalAI-VOXL2 or ModalAI-VOXL3"
    exit 1
fi

BUILDDIR="build/${BOARD}"

if [ ! -d "$BUILDDIR" ]; then
    echo "Error: Build directory $BUILDDIR not found"
    exit 1
fi

set -e

if $USE_ADB; then
    echo "Installing ArduPilot via adb from $BUILDDIR"
    adb push ${BUILDDIR}/bin/arducopter /usr/lib/rfsa/adsp/ArduPilot.so
    adb push ${BUILDDIR}/ardupilot /usr/bin/
else
    [ $# -eq 1 ] || {
        echo "Error: IPADDRESS is required when using rsync"
        usage
    }
    DEST="$1"
    echo "Installing ArduPilot on $DEST from $BUILDDIR"
    rsync -a ${BUILDDIR}/bin/arducopter $DEST:/usr/lib/rfsa/adsp/ArduPilot.so
    rsync -a ${BUILDDIR}/ardupilot $DEST:/usr/bin/
    echo "Restarting ArduPilot"
    ssh $DEST systemctl restart voxl-ardupilot
fi

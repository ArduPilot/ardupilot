#!/bin/bash
# script to install ArduPilot on a voxl2 board
# this assumes you have already installed the voxl-ardupilot.service file
# and /usr/bin/voxl-ardupilot script

USE_ADB=false

while getopts "a" opt; do
    case $opt in
        a)
            USE_ADB=true
            ;;
        *)
            echo "Usage: install.sh [-a] [IPADDRESS]"
            echo "  -a  Use adb push instead of rsync"
            exit 1
            ;;
    esac
done
shift $((OPTIND - 1))

set -e

if $USE_ADB; then
    echo "Installing ArduPilot via adb"
    adb push build/QURT/bin/arducopter /usr/lib/rfsa/adsp/ArduPilot.so
    adb push build/QURT/ardupilot /usr/bin/
else
    [ $# -eq 1 ] || {
        echo "Usage: install.sh [-a] IPADDRESS"
        exit 1
    }
    DEST="$1"
    echo "Installing ArduPilot on $DEST"
    rsync -a build/QURT/bin/arducopter $DEST:/usr/lib/rfsa/adsp/ArduPilot.so
    rsync -a build/QURT/ardupilot $DEST:/usr/bin/
    echo "Restarting ArduPilot"
    ssh $DEST systemctl restart voxl-ardupilot
fi

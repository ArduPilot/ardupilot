#!/bin/bash
# script to install ArduPilot on a voxl2 board
# this assumes you have already installed the voxl-ardupilot.service file
# and /usr/bin/voxl-ardupilot script

[ $# -eq 1 ] || {
    echo "install.sh IPADDRESS"
    exit 1
}

DEST="$1"

set -e

echo "Installing ArduPilot on $DEST"

rsync -a build/QURT/bin/arducopter $DEST:/usr/lib/rfsa/adsp/ArduPilot.so
rsync -a build/QURT/ardupilot $DEST:/usr/bin/

echo "Restarting ArduPilot"
ssh $DEST systemctl restart voxl-ardupilot

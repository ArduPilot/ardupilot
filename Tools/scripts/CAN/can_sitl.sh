#!/bin/bash
# this script sets up SITL to be able to attach to real CAN devices
# once run, you can configure SITL for CAN just like a real board, with the CAN parameters
#
# CAN_P1_DRIVER=1
# CAN_D1_PROTOCOL=1

# once running you can also attach uavcan_gui_tool to vcan0 to monitor the CAN bus

[ $# -eq 1 ] || {
    echo "Usage: can_sitl.sh DEVICE"
    echo "for example can_sitl.sh /dev/serial/by-id/usb-Zubax_Robotics_Zubax_Babel_3700330018514D563935392000000000-if00"
    exit 1
}

DEVPATH="$1"

if readlink $DEVPATH > /dev/null; then
    DEVNAME=$(basename $(readlink $DEVPATH))
else
    DEVNAME=$(basename $DEVPATH)
fi

set -e

# cleanup from a previous run
sudo killall -9 slcand 2> /dev/null || true
for m in slcan can-gw vcan can_raw can; do
    sudo rmmod $m 2> /dev/null || true
done

sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
sudo modprobe slcan
sudo modprobe can-gw
sudo slcan_attach -f -s8 -o "$DEVPATH"
sudo slcand "$DEVNAME" slcan0
sudo ifconfig slcan0 up
sudo cangw -A -s vcan0 -d slcan0 -e
sudo cangw -A -s slcan0 -d vcan0 -e

echo "slcan0 setup"
ifconfig slcan0

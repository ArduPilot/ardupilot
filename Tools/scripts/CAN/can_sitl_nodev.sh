#!/usr/bin/env bash

# once run, you can configure SITL for CAN just like a real board, with the CAN parameters
#
# CAN_P1_DRIVER=1
# CAN_D1_PROTOCOL=1

# once running you can also attach uavcan_gui_tool to vcan0 to monitor the CAN bus

set -e
set -x

# cleanup from a previous run
sudo killall -9 slcand 2> /dev/null || true
for m in slcan can-gw vcan can_raw can; do
    sudo rmmod $m 2> /dev/null || true
done

sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
sudo ip link set vcan0 mtu 72
sudo modprobe can-gw

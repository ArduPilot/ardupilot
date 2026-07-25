#!/bin/bash

# this sets up a vcan0 network for use with socketcan

set -e
set -x

sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
sudo ip link set vcan0 mtu 72


#!/bin/bash
# simple test of DSDL compilation for linux

set -e
set -x

# test compiler on linux
python3 -m pip install -U empy pexpect dronecan
rm -rf DSDL
git clone https://github.com/DroneCAN/DSDL

echo "Testing generation"
python3 dronecan_dsdlc.py --output dsdl_generated DSDL/dronecan DSDL/uavcan DSDL/com DSDL/ardupilot 

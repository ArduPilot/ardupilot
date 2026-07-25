#!/bin/bash
# test of DSDL compilation for linux with full regression test of all messages

set -e
set -x

# test compiler on linux
python3 -m pip install -U empy pexpect dronecan
rm -rf DSDL

git clone https://github.com/DroneCAN/DSDL
(cd .. && git clone https://github.com/DroneCAN/libcanard)

echo "Testing generation with regression testing"
python3 dronecan_dsdlc.py --output dsdl_generated DSDL/dronecan DSDL/uavcan DSDL/com DSDL/ardupilot --run-test

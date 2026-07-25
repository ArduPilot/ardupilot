#!/bin/bash
# test of DSDL compilation for linux with full regression test of all messages

set -e
set -x

# test compiler on linux
python3 -m pip install -U empy==3.3.4 pexpect dronecan

pushd ..
[ -d dronecan_dsdlc ] || git clone https://github.com/DroneCAN/dronecan_dsdlc
[ -d libcanard ] || git clone https://github.com/DroneCAN/libcanard
popd

echo "Testing generation with regression testing"
python3 ../dronecan_dsdlc/dronecan_dsdlc.py --output dsdl_generated dronecan uavcan com ardupilot --run-test

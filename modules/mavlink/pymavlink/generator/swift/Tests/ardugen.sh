#!/bin/sh

export PYTHONPATH="$PYTHONPATH:../../../../"

rm -rf ./MAVLink/MAVLink/Swift/
python3 -m pymavlink.tools.mavgen ./MAVLinkTests/Testdata/ardupilotmega.xml -o ./MAVLink/MAVLink/Swift/ --wire-protocol 1.0 --lang Swift

rm -rf ./MAVLinkTests/MAVLink/C/
python3 -m pymavlink.tools.mavgen ./MAVLinkTests/Testdata/ardupilotmega.xml -o ./MAVLinkTests/MAVLink/C/ --wire-protocol 1.0 --lang C

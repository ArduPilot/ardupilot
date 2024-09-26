#!/usr/bin/env bash

set -ex

./dv/scripts/test_connection.py
./dv/scripts/reboot_autopilot.py

sleep 5

./dv/scripts/test_connection.py

python3 ./Tools/scripts/uploader.py --port /dev/ttyTHS1 --baud-flightstack 921600,500000,115200,57600  --baud-bootloader 115200 --identify
python3 ./Tools/scripts/uploader.py --port /dev/ttyTHS1 --baud-flightstack 921600,500000,115200,57600  --baud-bootloader 115200 ./build/CubeOrangePlus-dv/bin/arducopter.apj

./dv/scripts/test_connection.py
./dv/scripts/request_default_params.py
./dv/scripts/test_connection.py

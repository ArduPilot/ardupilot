#!/usr/bin/env bash

set -ex

./dv/scripts/wait_online.py /dev/ttyTHS1
./dv/scripts/reboot_autopilot.py
./dv/scripts/wait_online.py /dev/ttyTHS1

python3 ./Tools/scripts/uploader.py --port /dev/ttyTHS1 --baud-flightstack 921600,500000,115200,57600  --baud-bootloader 115200 --identify
python3 ./Tools/scripts/uploader.py --port /dev/ttyTHS1 --baud-flightstack 921600,500000,115200,57600  --baud-bootloader 115200 ./build/CubeOrangePlus-dv/bin/arducopter.apj

./dv/scripts/wait_online.py /dev/ttyTHS1
./dv/scripts/request_default_params.py /dev/ttyTHS1
./dv/scripts/wait_online.py /dev/ttyTHS1

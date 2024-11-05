#!/usr/bin/env bash

set -ex

python3 ./Tools/scripts/uploader.py --baud-flightstack 921600,115200,57600  --baud-bootloader 115200 --identify
python3 ./Tools/scripts/uploader.py --baud-flightstack 921600,115200,57600  --baud-bootloader 115200 ./build/CubeOrangePlus-dv/bin/arducopter.apj

sleep 5

PORT="/dev/serial/by-id/$(ls /dev/serial/by-id | grep Cube | grep if00)"

./dv/scripts/wait_online.py "${PORT}"
./dv/scripts/request_default_params.py "${PORT}"
sleep 2
./dv/scripts/wait_online.py "${PORT}"

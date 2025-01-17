#!/usr/bin/env bash

set -ex

python3 ./Tools/scripts/uploader.py --baud-flightstack 921600,115200,57600  --baud-bootloader 115200 ./build/CubeOrangePlus-dv/bin/arducopter.apj

sleep 7

PORT="/dev/serial/by-id/$(ls /dev/serial/by-id | grep Cube | grep if00)"

./dv/scripts/wait_online.py "${PORT}"
./dv/scripts/request_flash_bootloader.py "${PORT}"
./dv/scripts/request_default_params.py "${PORT}"
sleep 5
./dv/scripts/wait_online.py "${PORT}"

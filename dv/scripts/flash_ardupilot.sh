#!/usr/bin/env bash

set -ex

PORT=/dev/ttyTHS1

./dv/scripts/wait_online.py "${PORT}"
./dv/scripts/reboot_autopilot.py "${PORT}"
./dv/scripts/wait_online.py "${PORT}"

python3 ./Tools/scripts/uploader.py --port "${PORT}" --baud-flightstack 921600,500000,115200,57600  --baud-bootloader 115200 --identify
python3 ./Tools/scripts/uploader.py --port "${PORT}" --baud-flightstack 921600,500000,115200,57600  --baud-bootloader 115200 ./build/CubeOrangePlus-dv/bin/arducopter.apj

./dv/scripts/wait_online.py "${PORT}"
./dv/scripts/request_default_params.py "${PORT}"
./dv/scripts/wait_online.py "${PORT}"

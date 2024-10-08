#!/usr/bin/env bash

set -ex

if [ -z "$1" ]
  then
    PORT=/dev/ttyACM0
    echo "No argument supplied, using default port: ${PORT}"
  else
    PORT="$1"
    echo "Flashing cube at: ${PORT}"
fi

python3 ./Tools/scripts/uploader.py --port "${PORT}" --baud-flightstack 921600,115200,57600  --baud-bootloader 115200 --identify
python3 ./Tools/scripts/uploader.py --port "${PORT}" --baud-flightstack 921600,115200,57600  --baud-bootloader 115200 ./build/CubeOrangePlus-dv/bin/arducopter.apj

./dv/scripts/wait_online.py "${PORT}"
./dv/scripts/request_default_params.py "${PORT}"
./dv/scripts/wait_online.py "${PORT}"

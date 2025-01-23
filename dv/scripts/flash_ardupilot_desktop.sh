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

CURRENT_REVISION=$(./dv/scripts/get_current_revision.py "${PORT}")
TARGET_REVISION=$(./dv/scripts/get_firmware_revision.py ./build/CubeOrangePlus-dv/bin/arducopter.apj)
if [ "$CURRENT_REVISION" == "$TARGET_REVISION" ]
then
  echo "all OK"
else
  echo -e "Revisions do not match"
  echo "CURRENT_REVISION: $CURRENT_REVISION"
  echo "TARGET_REVISION: $TARGET_REVISION"
fi

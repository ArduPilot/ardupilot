#!/usr/bin/env bash

set -ex

PORT=/dev/ttyTHS1

./dv/scripts/wait_online.py "${PORT}"
./dv/scripts/reboot_autopilot.py "${PORT}"
./dv/scripts/wait_online.py "${PORT}"

python3 ./Tools/scripts/uploader.py --port "${PORT}" --baud-flightstack 921600,500000,115200,57600  --baud-bootloader 115200 --identify
python3 ./Tools/scripts/uploader.py --port "${PORT}" --baud-flightstack 921600,500000,115200,57600  --baud-bootloader 115200 ./build/CubeOrangePlus-dv/bin/arducopter.apj

./dv/scripts/wait_online.py "${PORT}"
./dv/scripts/request_flash_bootloader.py "${PORT}"
./dv/scripts/request_default_params.py "${PORT}"
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

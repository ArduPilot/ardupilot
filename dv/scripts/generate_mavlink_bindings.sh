#!/usr/bin/env bash

if [ -z "$1" ]
then
    DIALECT=ardupilotmega
fi

OUTPUT=./build/mavlink_generated
DIALECT_PATH="./modules/mavlink/message_definitions/v1.0/${DIALECT}.xml"

mkdir -p ${OUTPUT}/${DIALECT}/c
mkdir -p ${OUTPUT}/${DIALECT}/cpp
mkdir -p ${OUTPUT}/${DIALECT}/wlua

python3 ./modules/mavlink/pymavlink/tools/mavgen.py --lang=C --wire-protocol=2.0 --output="${OUTPUT}/${DIALECT}/c" ${DIALECT_PATH}
python3 ./modules/mavlink/pymavlink/tools/mavgen.py --lang=C++11 --wire-protocol=2.0 --output="${OUTPUT}/${DIALECT}/cpp" "${DIALECT_PATH}"
python3 ./modules/mavlink/pymavlink/tools/mavgen.py --lang=WLua --wire-protocol=2.0 --output="${OUTPUT}/${DIALECT}/wlua/${DIALECT}" ${DIALECT_PATH}

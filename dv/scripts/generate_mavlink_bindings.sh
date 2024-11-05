#!/usr/bin/env bash

OUTPUT=./build/mavlink_generated

DIALECT=ardupilotmega

DIALECT_PATH="./modules/mavlink/message_definitions/v1.0/${DIALECT}.xml"

mkdir -p ${OUTPUT}/c/mavlink/v2.0
mkdir -p ${OUTPUT}/cpp/mavlink/v2.0
mkdir -p ${OUTPUT}/wlua

python3 ./modules/mavlink/pymavlink/tools/mavgen.py --lang=C --wire-protocol=2.0 --output="${OUTPUT}/c/mavlink/v2.0" ${DIALECT_PATH}
python3 ./modules/mavlink/pymavlink/tools/mavgen.py --lang=C++11 --wire-protocol=2.0 --output="${OUTPUT}/cpp/mavlink/v2.0" "${DIALECT_PATH}"
python3 ./modules/mavlink/pymavlink/tools/mavgen.py --lang=WLua --wire-protocol=2.0 --output="${OUTPUT}/wlua/${DIALECT}" ${DIALECT_PATH}


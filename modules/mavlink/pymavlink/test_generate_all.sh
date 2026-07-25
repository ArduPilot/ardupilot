#!/bin/bash
set -e
set -x

test -z "$MDEF" && MDEF="../message_definitions"

echo "Test Generate ALL in mavlink v1"
mavgen.py --lang='C'          --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=1.0 --strict-units
mavgen.py --lang='CS'         --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=1.0 --strict-units
mavgen.py --lang='JavaScript' --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=1.0 --strict-units
mavgen.py --lang='Python3'     --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=1.0 --strict-units
mavgen.py --lang='WLua'       --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=1.0 --strict-units
mavgen.py --lang='ObjC'       --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=1.0 --strict-units
mavgen.py --lang='Swift'      --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=1.0 --strict-units
mavgen.py --lang='Java'       --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=1.0 --strict-units
mavgen.py --lang='Lua'       --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=1.0 --strict-units


echo "Test Generate ALL in mavlink v2"
mavgen.py --lang='C'          --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units
mavgen.py --lang='CS'         --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units
mavgen.py --lang='JavaScript' --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units
mavgen.py --lang='TypeScript' --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units
mavgen.py --lang='Python3'     --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units
mavgen.py --lang='WLua'       --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units
mavgen.py --lang='ObjC'       --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units
mavgen.py --lang='Swift'      --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units
mavgen.py --lang='Java'       --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units
mavgen.py --lang='C++11'      --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units # C++11 generator only supports 2.0
mavgen.py --lang='Lua'       --output=/tmp/mavgen_test mavlink/message_definitions/v1.0/common.xml --wire-protocol=2.0 --strict-units

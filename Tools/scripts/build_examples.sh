#!/bin/bash

set -e

cat >&2 <<EOF
This script is deprecated in favour of running waf with 'examples' as the main command

cd \$ARDUPILOT_HOME
./modules/waf/waf-light configure --board=linux
./modules/waf/waf-light examples

Sleeping for a few seconds to let you digest that.
EOF

sleep 4

set -x

PY="$(dirname $0)/build_examples.py"

$PY $*

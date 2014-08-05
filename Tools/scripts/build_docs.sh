#!/bin/bash

set -e

# work from either APM directory or above
[ -d ArduPlane ] || cd APM

export DOCS_OUTPUT_BASE=../buildlogs/docs

(
./docs/build-libs.sh
./docs/build-arduplane.sh
./docs/build-arducopter.sh
./docs/build-apmrover2.sh
) > ../buildlogs/build_docs.log 2>&1

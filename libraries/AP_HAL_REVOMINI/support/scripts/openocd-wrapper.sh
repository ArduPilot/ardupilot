#!/usr/bin/env bash

# Helper to decide which openocd script to use. We only support 0.3.x and 0.4.x.

if [ $# -ne 1 ]
then
  echo "Usage: `basename $0` {flash|debug}"
  exit 1
fi

OPENOCD_VERSION=`openocd -v 2>&1 | head -n1 | \
                 awk '{print $4}' | sed 's/\([0-9]*\.[0-9]*\)\.[0-9]*/\1/'`

CFG_FILE=$1_${OPENOCD_VERSION}.cfg

openocd -f support/openocd/$CFG_FILE

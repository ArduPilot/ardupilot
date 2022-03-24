#!/bin/bash

set -e
set -x

BOARD=NucleoH743
BOARD=MatekH743
#BOARD=F35Lightning

THISDIR=$(dirname $0)

./waf configure \
      --board=$BOARD \
      --extra-hwdef="$THISDIR/extra-hwdef-sitl-on-hw.dat" \
      --default-param="$THISDIR/default.param"

./waf copter --upload

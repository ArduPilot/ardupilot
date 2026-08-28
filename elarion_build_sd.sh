#!/bin/bash
set -e
source .venv/bin/activate
export PATH="$HOME/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH"
./waf configure --board ELARIONH7V1SD
./waf copter

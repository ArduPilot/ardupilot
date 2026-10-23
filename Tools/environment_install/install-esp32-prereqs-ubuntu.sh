#!/bin/bash

set -e
set -x

sudo apt-get install -y git wget flex bison gperf cmake ninja-build ccache libffi-dev libssl-dev dfu-util
./Tools/scripts/esp32_get_idf.sh

cd modules/esp_idf
# use it:
./install.sh
unset IDF_PATH

echo "source $PWD/export.sh" >>~/.bashrc

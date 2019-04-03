#!/bin/bash
export IDF_PATH=/home/buzz/esp/esp-idf
export PATH=/home/buzz/esp/xtensa-esp32-elf/bin:/usr/lib/ccache:/home/buzz/ardupilot/Tools/autotest:/opt/gcc-arm-none-eabi-4_9-2015q3/bin:/home/buzz/.local/share/umake/bin:/home/buzz/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin:/home/buzz/jsbsim/src:/home/buzz/.local/bin/

cd ~/ardupilot
./waf $1 $2

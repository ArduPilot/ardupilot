#!/bin/bash

# esp32 cmake+scripting has a weird bug right now where it won't build with ccache enabled, so for now --disable-scripting is a work-around to allow faster ccache-based builds to work.

# on my computer, a typical complete esp32 build without ccache takes 20+ minutes, 
# and after a 'rm -rf build' and with a primed ccache, it takes 4 minutes, so if iterating on esp32 a lot, its worth
# disabling scripting and enabling ccache for turning bulds around faster.


unset IDF_CCACHE_ENABLE
source ./modules/esp_idf/export.sh
unset CXX
unset CC

# plane: ---------------------------------------

rm -rf build

#./waf configure --board=esp32s3buzz --debug --toolchain=xtensa-esp32s3-elf --disable-scripting
./waf configure --board=esp32s3buzz --debug --disable-scripting
#./waf configure --board f103-GPS
echo "about to build PLANE for ESP32-S3 in 3 sec..."
sleep 3

#with ccache, but without scripting, its ok
time ESPBAUD=921600 ./waf plane --jobs=7 --upload
# -v -v

#periph: ---------------------------------------
#rm -rf build

./waf configure --board=esp32s3buzz_periph --debug --disable-scripting
#./waf configure --board f103-GPS
echo "about to build AP_Periph for ESP32-S3 in 3 sec..."
sleep 3

#with ccache, but without scripting, its ok
time ESPBAUD=921600 ./waf AP_Periph --jobs=7 --upload
# -v -v


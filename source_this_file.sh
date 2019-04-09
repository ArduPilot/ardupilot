export IDF_PATH=/home/buzz/ardupilot/modules/esp_idf
# also, which we are here, define the version we're building in version.txt for the IDF, or it gets it wrong:
grep THISFIRMWARE ArduPlane/version.h | cut -d\" -f 2 | sed -e "s/\s/-/g" >  libraries/AP_HAL_ESP32/plane/version.txt
grep THISFIRMWARE ArduCopter/version.h | cut -d\" -f 2 | sed -e "s/\s/-/g" >  libraries/AP_HAL_ESP32/copter/version.txt
grep THISFIRMWARE APMrover2/version.h | cut -d\" -f 2 | sed -e "s/\s/-/g" >  libraries/AP_HAL_ESP32/rover/version.txt

#!/bin/sh

/usr/local/stlink/st-flash  --reset read  eeprom.bin 0x08004000  0xc000 && \
/usr/local/stlink/st-flash  --reset write ../../../../../ArduPlane/f4light_Revolution_EE128_bl.bin 0x08000000
/usr/local/stlink/st-flash  --reset write eeprom.bin 0x08004000
/usr/local/stlink/st-util -m
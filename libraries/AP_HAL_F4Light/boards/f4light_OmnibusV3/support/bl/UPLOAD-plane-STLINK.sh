#!/bin/sh

# production binary with bootloader
#/usr/local/stlink/st-flash  --reset write /tmp/ArduCopter.build/f4light_Revolution.bin 0x08010000

#bare metal binary
/usr/local/stlink/st-flash  --reset read  eeprom.bin 0x08004000  0xc000 && \
/usr/local/stlink/st-flash  --reset write ../../../../../../ArduPlane/f4light_OmnibusV3_bl.bin 0x08000000
/usr/local/stlink/st-flash  --reset write eeprom.bin 0x08004000
/usr/local/stlink/st-util -m



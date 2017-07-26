#!/bin/sh

# production binary with bootloader
#/usr/local/stlink/st-flash  --reset write /tmp/ArduCopter.build/revomini_MP32V1F4.bin 0x08010000

#bare metal binary
/usr/local/stlink/st-flash  --reset write /usr/src/arduino/build/test-wayback.ino.bin  0x08000000 && /usr/local/stlink/st-util -m



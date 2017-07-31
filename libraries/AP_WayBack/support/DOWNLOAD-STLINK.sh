#!/bin/sh

# production binary with bootloader
#/usr/local/stlink/st-flash  --reset write /tmp/ArduCopter.build/revomini_MP32V1F4.bin 0x08010000

#bare metal binary
/usr/local/stlink/st-flash  --reset read read.bin  0x08000000 0x80000



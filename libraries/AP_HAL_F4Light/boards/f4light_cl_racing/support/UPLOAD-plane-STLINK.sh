#!/bin/sh

# production binary with bootloader
#/usr/local/stlink/st-flash  --reset write /tmp/ArduCopter.build/f4light_Revolution.bin 0x08010000

#bare metal binary
/usr/local/stlink/st-flash  --reset write ../../../../../ArduPlane/f4light_cl_racing.bin 0x08000000 && /usr/local/stlink/st-util -m



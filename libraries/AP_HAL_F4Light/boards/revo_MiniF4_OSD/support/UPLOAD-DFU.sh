#!/bin/sh

#production binary for bootloader
#dfu-util -a 0 --dfuse-address 0x08010000 -D /tmp/ArduCopter.build/revomini_AirbotV2.bin

# bare metal binary
dfu-util -a 0 --dfuse-address 0x08000000 -D ../../../../../ArduCopter/MiniF4_OSD.bin


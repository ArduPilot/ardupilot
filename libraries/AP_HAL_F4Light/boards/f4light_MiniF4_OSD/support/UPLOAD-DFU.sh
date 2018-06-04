#!/bin/sh

#production binary for bootloader
#dfu-util -a 0 --dfuse-address 0x08010000 -D /tmp/ArduCopter.build/f4light_AirbotV2.bin

# bare metal binary
dfu-util -a 0 --dfuse-address 0x08000000 -D ../../../../../ArduCopter/f4light_MiniF4_OSD.bin


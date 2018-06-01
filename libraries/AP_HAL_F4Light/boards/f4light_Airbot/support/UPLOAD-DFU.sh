#!/bin/sh

#production binary for bootloader
dfu-util -a 0 --dfuse-address 0x08010000:leave -D ../../../../../ArduCopter/f4light_Airbot.bin

# bare metal binary
#dfu-util -a 0 --dfuse-address 0x08000000:unprotect:force -D ../../../../../ArduCopter/f4light_Airbot.bin


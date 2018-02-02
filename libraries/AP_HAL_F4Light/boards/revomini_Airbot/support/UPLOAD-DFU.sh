#!/bin/sh

#production binary for bootloader
#dfu-util -a 0 --dfuse-address 0x08010000 -D /tmp/ArduCopter.build/revomini_Revolution.bin

# bare metal binary
#dfu-util -a 0 --dfuse-address 0x08000000:unprotect:force -D ../../../../../ArduCopter/revomini_Airbot.bin
dfu-util -a 0 --dfuse-address 0x08000000:unprotect:force -D ../../../../../ArduCopter/revomini_Airbot.bin


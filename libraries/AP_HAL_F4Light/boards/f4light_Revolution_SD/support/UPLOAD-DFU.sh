#!/bin/sh

#production binary for bootloader
#dfu-util -a 0 --dfuse-address 0x08010000 -D /tmp/ArduCopter.build/f4light_Revolution.bin

# bare metal binary
#dfu-util -a 0 --dfuse-address 0x08000000:unprotect:force -D /tmp/ArduCopter.build/f4light_Revolution.bin
dfu-util -a 0 --dfuse-address 0x08010000:leave -D ../../../../../ArduCopter/f4light_Revolution_SD.bin -R


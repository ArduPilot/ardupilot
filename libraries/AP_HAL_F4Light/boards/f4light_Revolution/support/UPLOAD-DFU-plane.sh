#!/bin/sh

# bare metal binary
#dfu-util -a 0 --dfuse-address 0x08000000:unprotect:force -D /tmp/ArduCopter.build/f4light_Revolution.bin

#production binary for bootloader
dfu-util -a 0 --dfuse-address 0x08010000:leave -D ../../../../../ArduPlane/f4light_Revolution.bin -R


#!/bin/sh

# bare metal binary
dfu-util -a 0 --dfuse-address 0x08000000 -D /usr/src/arduino/build/test-wayback.ino.bin 


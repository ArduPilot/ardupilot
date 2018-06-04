#!/bin/sh

dfu-util -a 0 --dfuse-address 0x08004000:leave -Z 0xc000 -U $1.bin  -R

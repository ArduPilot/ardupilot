#!/bin/sh

/usr/local/stlink/st-flash  --reset read  $1.bin 0x08004000  0xc000



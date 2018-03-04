#!/bin/sh
/usr/local/stlink/st-flash  --reset write $1.bin 0x08004000 



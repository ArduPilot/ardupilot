#!/bin/sh


/usr/local/stlink/st-flash  --reset write ../../../../../ArduPlane/f4light_MatekF405-wing.bin 0x08010000 && \
/usr/local/stlink/st-util -m



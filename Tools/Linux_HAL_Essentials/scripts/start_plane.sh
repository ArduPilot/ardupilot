#!/bin/bash

cd /root
(
    date
    init 3
    killall -q udhcpd
    (cd Linux_HAL_Essentials && ./startup.sh load)
    while :; do
	./ArduPlane.elf -A /dev/ttyO0 -B /dev/ttyO5
    done
) >> plane.log 2>&1


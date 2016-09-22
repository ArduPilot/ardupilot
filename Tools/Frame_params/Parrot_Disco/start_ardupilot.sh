#!/bin/sh

cd /data/ftp/internal_000/APM
(
 date

 # startup fan
 echo 1 > /sys/devices/platform/user_gpio/FAN/value
 
 while :; do
    echo "$(date) Starting arduplane"
   ./arduplane -A udp:192.168.42.255:14550:bcast -B /dev/ttyPA1 -C udp:192.168.43.255:14550:bcast --module-directory modules
 done
) >> start_ardupilot.log 2>&1 &




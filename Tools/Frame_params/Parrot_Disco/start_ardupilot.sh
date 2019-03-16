#!/bin/sh

cd /data/ftp/internal_000/ardupilot
(
 /bin/date
 /bin/ls

 /bin/dragon_ipc.sh dragon_shutdown
 /bin/sleep 2
 /usr/bin/killall -KILL dragon-prog
 /bin/sleep 1

 echo "step2"
 # stop stock led daemon
 /usr/bin/pstop ledd

 # setup for video
 /usr/bin/media-ctl -l '"mt9f002 0-0010":0->"avicam.0":0[1]'
 /usr/bin/media-ctl -l '"avicam_dummy_dev.0":0->"avicam.0":0[0]'
 /usr/bin/prestart dxowrapperd
 /usr/bin/prestart pimp

 echo "step3"

 # startup fan
 echo 1 > /sys/devices/platform/user_gpio/FAN/value

 # setup GPS
 echo 1 > /sys/devices/platform/user_gpio/RESET_GNSS/value
 /bin/sleep 1
 echo 0 > /sys/devices/platform/user_gpio/RESET_GNSS/value

 echo "step4"

 while :; do
    echo "$(date) Starting arduplane"
   ./arduplane -A udp:192.168.42.255:14550:bcast -B /dev/ttyPA1 -C udp:192.168.43.255:14550:bcast --module-directory modules
 done
) >> start_ardupilot.log 2>&1

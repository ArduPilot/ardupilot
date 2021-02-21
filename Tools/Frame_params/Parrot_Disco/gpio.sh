#!/bin/sh
# this is a script triggered from GPIO changes. It is setup to take photos
# start/stop recording and start/stop streaming on a disco

PIN="$1"
VALUE="$2"
echo "got pin=$PIN value=$VALUE"

PATH=$PATH:/bin:/usr/bin:/data/ftp/internal_000/ardupilot
export PATH

cd /data/ftp/internal_000/ardupilot

if [ $PIN = 100 ]; then
    # take photo when high
    if [ $VALUE = 1 ]; then
        echo "$(date) Taking picture" >> gpio.log
        /usr/bin/pimpctl take-picture front
    fi
fi

if [ $PIN = 101 ]; then
    # recording start/stop
    if [ $VALUE = 1 ]; then
        echo "$(date) Starting recording" >> gpio.log
        /usr/bin/pimpctl recording-start front
    else
        echo "$(date) Stopping recording" >> gpio.log
        /usr/bin/pimpctl recording-stop front
    fi
fi

if [ $PIN = 102 ]; then
    GCS_IP=$(netstat -n|grep 14550 | head -1 | awk '{print $5}'| cut -d: -f1)
    # streaming start/stop
    if [ $VALUE = 1 ]; then
        echo "$(date) Starting streaming to $GCS_IP 8888" >> gpio.log
        /usr/bin/pimpctl stream-start front $GCS_IP 8888
    else
        echo "$(date) Stopping streaming to $GCS_IP 8888" >> gpio.log
        /usr/bin/pimpctl stream-stop front $GCS_IP 8888
    fi
fi

#!/bin/bash
# an example script that starts udpproxy for multiple ports under GNU
# screen, allowing for unattended operation of the proxy for long
# periods

killall -9 udpproxy
screen -AdmS proxy -t tab0 bash

BASE_PORT=10401
NUM_PORTS=10
port=$BASE_PORT
count=$NUM_PORTS
while [ $count -gt 0 ]; do
    port2=$((port+1))
    echo $port $port2
    screen -S proxy -X screen -t $port ./udpproxy $port $port2 -v
    port=$((port+2))
    count=$((count-2))
done

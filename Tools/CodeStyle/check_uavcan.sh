#!/usr/bin/env bash

FOLDER=../../libraries/AP_UAVCAN_V1/*

for filename in $FOLDER; do
    echo $filename
    ./ardupilot-astyle.sh $filename
done

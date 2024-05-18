#!/bin/bash

# Author: Andrew Gregg

# This script is used to flash the ArduRover firmware via a remote machine, such as
# a companion computer. It copies the uploader.py script and the contents of the 
# ARDUROVER_DIR to the remote machine and executes the uploader.py script to flash 
# the firmware.

# The script takes three arguments:
# 1. IP_ADDRESS: The IP address of the remote machine.
# 2. PASSWORD: The password of the remote machine.
# 3. ARDUROVER_DIR: The directory containing the ArduRover firmware files.

# Check if the correct number of arguments are provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <IP_ADDRESS> <PASSWORD> <ARDUROVER_DIR>"
    exit 1
fi

# Assign arguments to variables
IP_ADDRESS=$1
PASSWORD=$2
ARDUROVER_DIR=$3

# Define the remote directory
REMOTE_DIR="/home/havoc/ardurover_flash"

# Check if the ARDUROVER_DIR exists and is a directory
if [ ! -d "$ARDUROVER_DIR" ]; then
    echo "Error: Directory $ARDUROVER_DIR does not exist."
    exit 1
fi

# Check if the remote directory exists and clear it if it does
sshpass -p "$PASSWORD" ssh "$IP_ADDRESS" << EOF
    if [ -d "$REMOTE_DIR" ]; then
        rm -rf "$REMOTE_DIR"/*
    else
        mkdir -p "$REMOTE_DIR"
    fi
EOF

# Copy uploader.py and contents of ARDUROVER_DIR to the remote machine
sshpass -p "$PASSWORD" scp uploader.py "$IP_ADDRESS:$REMOTE_DIR/"
sshpass -p "$PASSWORD" scp -r "$ARDUROVER_DIR"/* "$IP_ADDRESS:$REMOTE_DIR/"

# Run the uploader.py script on the remote machine
sshpass -p "$PASSWORD" ssh "$IP_ADDRESS" << EOF
    cd "$REMOTE_DIR"
    python3 uploader.py ardurover.apj
EOF

echo "Files transferred and uploader.py executed successfully on $IP_ADDRESS"
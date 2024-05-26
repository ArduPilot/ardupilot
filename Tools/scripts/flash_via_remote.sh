#!/bin/bash

# Enable exit on error
set -e

# Author: Andrew Gregg

# This script is designed to flash the ArduRover firmware onto a remote machine, such as a companion computer.
# It first copies the uploader.py script and the contents of the ARDUROVER_DIR to the remote machine.
# Then, it executes the uploader.py script on the remote machine to flash the firmware.

# The script requires three arguments:
# 1. IP_ADDRESS: The IP address of the remote machine, in the format username@ip_address.
# 2. PASSWORD: The password for the remote machine.
# 3. ARDUROVER_DIR: The directory containing the ArduRover firmware files. If not provided, defaults to '../../build/CubeOrangePlus/bin'.

# Check if sshpass is installed, if not, offer to install it
if ! command -v sshpass &> /dev/null
then
    echo "sshpass could not be found"
    read -p "Do you want to install sshpass? (y/n) " -n 1 -r
    echo    # move to a new line
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
        sudo apt-get install sshpass
    else
        echo "sshpass is required for this script to run. Exiting."
        exit 1
    fi
fi

# Check if the correct number of arguments are provided, if not, display usage and exit
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <IP_ADDRESS> <PASSWORD> [<ARDUROVER_DIR>]"
    exit 1
fi

# Check if the IP address argument contains a username, if not, display warning and exit
if [[ $1 != *@* ]]; then
    echo -e "\e[1;33mWarning: IP address argument does not contain a username. It should be in the format username@ip_address.\e[0m"
    exit 1
fi

# Assign arguments to variables
IP_ADDRESS=$1
PASSWORD=$2
ARDUROVER_DIR=${3:-../../build/CubeOrangePlus/bin}

# Define the remote directory
REMOTE_DIR="/home/havoc/ardurover_flash"

# Check if the ARDUROVER_DIR exists and is a directory, if not, display error and exit
if [ ! -d "$ARDUROVER_DIR" ]; then
    echo "Error: Directory $ARDUROVER_DIR does not exist."
    exit 1
fi

# Check if the remote directory exists and clear it if it does, or create it if it doesn't
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no "$IP_ADDRESS" << EOF
    if [ -d "$REMOTE_DIR" ]; then
        rm -rf "$REMOTE_DIR"/*
    else
        mkdir -p "$REMOTE_DIR"
    fi
EOF

# Check the exit status of the ssh command, if it failed, display error and exit
if [ $? -ne 0 ]; then
    echo "Failed to execute ssh command. Exiting."
    exit 1
fi

# Copy uploader.py and contents of ARDUROVER_DIR to the remote machine
sshpass -p "$PASSWORD" scp -o StrictHostKeyChecking=no uploader.py "$IP_ADDRESS:$REMOTE_DIR/"
sshpass -p "$PASSWORD" scp -o StrictHostKeyChecking=no -r "$ARDUROVER_DIR"/* "$IP_ADDRESS:$REMOTE_DIR/"

# Check the exit status of the scp commands, if they failed, display error and exit
if [ $? -ne 0 ]; then
    echo "Failed to copy files. Exiting."
    exit 1
fi

# Run the uploader.py script on the remote machine
sshpass -p "$PASSWORD" ssh -o StrictHostKeyChecking=no "$IP_ADDRESS" << EOF
    cd "$REMOTE_DIR"
    python3 uploader.py ardurover.apj
EOF

# Check the exit status of the ssh command, if it failed, display error and exit
if [ $? -ne 0 ]; then
    echo "Failed to execute uploader.py. Exiting."
    exit 1
fi

# If all steps were successful, display success message
echo "Files transferred and uploader.py executed successfully on $IP_ADDRESS"
#!/bin/bash

cleanup() {
    echo "Terminating background processes..."
    kill -9 $pid1 $pid2 2>/dev/null
    exit 1
}

trap cleanup SIGINT

# Start the first Python script in the background
python3 tracking.py &

# Capture the process ID of the first script
pid1=$!

# Start the second Python script in the background
python3 send_camera_information.py --sysid 245 --compid 0 --resh 640 --resv 480 &

# Capture the process ID of the second script
pid2=$!

# Wait for both scripts to complete
wait $pid1 $pid2

echo "Scripts Ended Cleanly"

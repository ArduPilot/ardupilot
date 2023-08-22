#!/bin/bash
# Build and run the motors example stability test at a range of yaw headroom and throttle average max values
# Output results to files for comparison

cd "$(dirname "$0")"
cd ../../../..

mkdir -p MotorTestSweep

./waf configure --board linux
./waf build --target examples/AP_Motors_test
echo

YAW_HEADROOM="0 100 200 300 400 500 600 700 800 900 1000"
THR_AVERAGE_MAX="0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0"

# Change this to change the frame classes to test. Testing all takes a while.
FRAME_CLASS="1 2 3 6 11 13"

COUNTER=0
for frame_class in $FRAME_CLASS; do
    echo "Frame Class: $frame_class"
    for headroom in $YAW_HEADROOM; do
        echo "Yaw Headroom: $headroom"
        for Thr in $THR_AVERAGE_MAX; do
            echo "    Throttle average max: $Thr"
            # Test with and without boost
            ./build/linux/examples/AP_Motors_test s frame_class=$frame_class yaw_headroom=$headroom throttle_avg_max=$Thr thrust_boost=0 > MotorTestSweep/$COUNTER.csv
            let COUNTER++
            ./build/linux/examples/AP_Motors_test s frame_class=$frame_class yaw_headroom=$headroom throttle_avg_max=$Thr thrust_boost=1 > MotorTestSweep/$COUNTER.csv
            let COUNTER++
        done
        echo
    done
done


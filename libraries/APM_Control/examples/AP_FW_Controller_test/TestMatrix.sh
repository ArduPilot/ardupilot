#!/usr/bin/env bash
# Fixedwing controller test and run over a range of inputs
# Output results to files for comparison

cd "$(dirname "$0")"
cd ../../../..

mkdir -p FW_Controller_matrix

./waf configure --board linux
./waf build --target examples/AP_FW_Controller_test
echo

Axis="roll pitch"
Angle="-20 10 5 0 5 10 20"
Airspeed="5 10 15 20 25"

COUNTER=0
# Range of roll angles
for roll in $Angle; do
    # Range of pitch angles
    for pitch in $Angle; do
        # Both roll and piith
        for ax in $Axis; do
            # Range of airspeeds
            for speed in $Airspeed; do
                ./build/linux/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch airspeed=$speed > FW_Controller_matrix/$COUNTER.csv
                let COUNTER++
            done

            # Airspeed failure
            ./build/linux/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch airspeed_fail=1 > FW_Controller_matrix/$COUNTER.csv
            let COUNTER++

            # Ground mode and intergrator diable flags, only test at defualt airspeed
            ./build/linux/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch ground_mode=1 > FW_Controller_matrix/$COUNTER.csv
            let COUNTER++

            ./build/linux/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch disable_integrator=1 > FW_Controller_matrix/$COUNTER.csv
            let COUNTER++

            ./build/linux/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch ground_mode=1 disable_integrator=1 > FW_Controller_matrix/$COUNTER.csv
            let COUNTER++
        done
    done
done


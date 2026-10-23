#!/usr/bin/env bash
# Fixedwing controller test and run over a range of inputs
# Output results to files for comparison

cd "$(dirname "$0")"
cd ../../../..

mkdir -p FW_Controller_matrix

./waf configure --board sitl
./waf build --target examples/AP_FW_Controller_test
echo

Axis="roll pitch"
RollAngle="-170 -160 -150 -140 -130 -120 -110 -100 -90 -80 -70 -60 -50 -40 -30 -20 10 0 10 20 30 40 50 60 70 80 90 100 110 120 130 140 150 160 170 180"
PitchAngle="-80 -70 -60 -50 -40 -30 -20 10 0 10 20 30 40 50 60 70 80"
Airspeed="5 10 15 20 25"

COUNTER=0
# Range of roll angles
for roll in $RollAngle; do
    # Range of pitch angles
    for pitch in $PitchAngle; do
        # Both roll and piith
        for ax in $Axis; do
            # Range of airspeeds
            for speed in $Airspeed; do
                ./build/sitl/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch airspeed=$speed > FW_Controller_matrix/$COUNTER.csv
                let COUNTER++
            done

            # Airspeed failure
            ./build/sitl/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch airspeed_fail=1 > FW_Controller_matrix/$COUNTER.csv
            let COUNTER++

            # Ground mode and intergrator diable flags, only test at defualt airspeed
            ./build/sitl/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch ground_mode=1 > FW_Controller_matrix/$COUNTER.csv
            let COUNTER++

            ./build/sitl/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch disable_integrator=1 > FW_Controller_matrix/$COUNTER.csv
            let COUNTER++

            ./build/sitl/examples/AP_FW_Controller_test axis=$ax roll=$roll pitch=$pitch ground_mode=1 disable_integrator=1 > FW_Controller_matrix/$COUNTER.csv
            let COUNTER++
        done
    done
done


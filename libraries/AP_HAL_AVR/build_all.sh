#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

set -e
set -x

HAL_AVR_TESTS="I2CDriver_HMC5883L"
LIBRARIES_TESTS="AP_ADC/examples/AP_ADC_test AP_Baro/examples/AP_Baro_MS5611_test"

echo "building AP_HAL examples"
pushd libraries/AP_HAL_AVR
for b in $HAL_AVR_TESTS; do
    pwd
    pushd examples/$b
    make clean
    make
    popd
done
popd

echo "building libraries examples"
pushd libraries
for b in $LIBRARIES_TESTS; do
    pwd
    pushd $b
    make clean
    make
    popd
done
popd

exit 0

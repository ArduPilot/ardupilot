#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

set -e
set -x

export PATH=/usr/lib/ccache:$PATH

HAL_AVR_TESTS="I2CDriver_HMC5883L AnalogIn APM1 Blink Console DataflashTest FastSerial LCDTest RCInputTest RCPassthroughTest Scheduler SPIDriver_MPU6000 Storage"
#HAL_AVR_TESTS=""

LIBRARIES_TESTS="AP_ADC/examples/AP_ADC_test AP_Baro/examples/AP_Baro_MS5611_test AP_Baro/examples/AP_Baro_BMP085_test AP_GPS/examples/GPS_AUTO_test AP_GPS/examples/GPS_UBLOX_test AP_GPS/examples/GPS_MTK_test AC_PID/examples/AC_PID_test AP_Airspeed/examples/Airspeed AP_Compass/examples/AP_Compass_test AP_Declination/examples/AP_Declination_test AP_InertialSensor/examples/MPU6000 AP_LeadFilter/examples/AP_LeadFilter AP_Math/examples/eulers AP_Math/examples/location AP_Math/examples/polygon AP_Math/examples/rotations Filter/examples/Derivative Filter/examples/Filter Filter/examples/LowPassFilter GCS_Console/examples/Console GCS_Console/examples/Console"

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

#!/bin/bash

# This script enables the Chip Select (since it's negated it should be put to low value)
# of the sensor passed as a parameter

MPU6000_CS_PIN=113 # Corresponds with P9_28
MPU9250_CS_PIN=49 # Corresponds with P9_28
MS5611_CS_PIN=7 # Corresponds with P9_42

# activate all the GPIOs and force them to untake the bus
echo "Disabling MPU6000 CS"
echo $MPU6000_CS_PIN > /sys/class/gpio/export 2> /dev/null
echo out > /sys/class/gpio/"gpio"$MPU6000_CS_PIN/direction
echo 1 > /sys/class/gpio/"gpio"$MPU6000_CS_PIN/value

echo "Disabling MPU9250 CS"
echo $MPU9250_CS_PIN > /sys/class/gpio/export 2> /dev/null
echo out > /sys/class/gpio/"gpio"$MPU9250_CS_PIN/direction
echo 1 > /sys/class/gpio/"gpio"$MPU9250_CS_PIN/value

echo "Disabling MS5611 CS"
echo $MS5611_CS_PIN > /sys/class/gpio/export 2> /dev/null
echo out > /sys/class/gpio/"gpio"$MS5611_CS_PIN/direction
echo 1 > /sys/class/gpio/"gpio"$MS5611_CS_PIN/value


if [ $# -eq 0 ]
  then
    	echo "No arguments supplied. Please provide one of the following sensors: mpu6000, mpu9250, ms5611"
	echo "   source enable_cs.sh <sensor> "
	return 0
fi

if [ $1 == "mpu6000" ]
then
	CS_PIN=$MPU6000_CS_PIN
	echo out > /sys/class/gpio/"gpio"$CS_PIN/direction
	echo 0 > /sys/class/gpio/"gpio"$CS_PIN/value
	echo "Enabling MPU6000 CS"
elif [ $1 == "mpu9250" ]
then
	CS_PIN=$MPU9250_CS_PIN
	echo out > /sys/class/gpio/"gpio"$CS_PIN/direction
	echo 0 > /sys/class/gpio/"gpio"$CS_PIN/value
	echo "Enabling MPU9250 CS"
elif [ $1 == "ms5611" ]
then
	CS_PIN=$MS5611_CS_PIN
	echo out > /sys/class/gpio/"gpio"$CS_PIN/direction
	echo 0 > /sys/class/gpio/"gpio"$CS_PIN/value
	echo "Enabling MS5611 CS"
else
    	echo "Sensor supplied invaled. Please provide one of the following sensors: mpu6000, mpu9250, ms5611"
	echo "   source enable_cs.sh <sensor> "
	return 0
fi

# to verify do:
#    cat /sys/class/gpio/"gpio"$CS_PIN/value

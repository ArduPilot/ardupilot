#!/bin/bash

# This script checks the Chip Select (since it's negated it should be put to low value)

MPU6000_CS_PIN=113 # Corresponds with P9_28
MPU9250_CS_PIN=49 # Corresponds with P9_28
MS5611_CS_PIN=7 # Corresponds with P9_42

# activate all the GPIOs and force them to untake the bus
echo "Checking MPU6000 CS"
cat /sys/class/gpio/"gpio"$MPU6000_CS_PIN/value

echo "Cheking MPU9250 CS"
cat /sys/class/gpio/"gpio"$MPU9250_CS_PIN/value

echo "Checking MS5611 CS"
cat /sys/class/gpio/"gpio"$MS5611_CS_PIN/value

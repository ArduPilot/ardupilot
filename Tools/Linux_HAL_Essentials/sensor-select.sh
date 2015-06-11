#!/bin/bash

# This script allows you to select which sensors you can use. For now it's resctricted to IMU use
#		Coded by Víctor Mayoral Vilches <victor@erlerobot.com>

IMU_CONFIG=$(grep -A 5 "#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE" ../../libraries/AP_HAL/AP_HAL_Boards.h| grep HAL_INS_DEFAULT)
echo "Current setup is: "$IMU_CONFIG

if [ $# -eq 0 ]
  then
    	echo "No arguments supplied. Please provide one of the following sensors: mpu6000, mpu9250, lsm9ds0"
	echo "   Usage: source sensor-select.sh <sensor> "
	return 0
fi

if [ $1 == "mpu6000" ]
then
	sed -i "s/$IMU_CONFIG/#define HAL_INS_DEFAULT HAL_INS_MPU6000/g" ../../libraries/AP_HAL/AP_HAL_Boards.h
	echo "MPU6000 selected"
elif [ $1 == "mpu9250" ]
then
	sed -i "s/$IMU_CONFIG/#define HAL_INS_DEFAULT HAL_INS_MPU9250/g" ../../libraries/AP_HAL/AP_HAL_Boards.h
	echo "MPU9250 selected"
elif [ $1 == "lsm9ds0" ]
then
	sed -i "s/$IMU_CONFIG/#define HAL_INS_DEFAULT HAL_INS_LSM9DS0/g" ../../libraries/AP_HAL/AP_HAL_Boards.h
	echo "LSM9DS0 selected"
else
    	echo "Sensor supplied invaled. Please provide one of the following sensors: mpu6000, mpu9250, lsm9ds0"
	echo "   Usage: source sensor-select.sh <sensor> "
	return 0
fi


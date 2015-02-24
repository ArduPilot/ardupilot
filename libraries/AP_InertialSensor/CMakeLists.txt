cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( DIR
  LSM9DS0
)

SET( AP_InertialSensor
  AP_InertialSensor
  AP_InertialSensor_Backend
  AP_InertialSensor_Flymaple
  AP_InertialSensor_HIL
  AP_InertialSensor_L3G4200D
  AP_InertialSensor_L3GD20
  AP_InertialSensor_LSM303D
  AP_InertialSensor_MPU6000
  AP_InertialSensor_MPU9150
  AP_InertialSensor_Oilpan
  AP_InertialSensor_PX4
  AP_InertialSensor_UserInteract_MAVLink
  AP_InertialSensor_UserInteract_Stream
)

include_directories( ${DIR} )
add_subdirectory( ${DIR} )

ADD_LIBRARY (AP_InertialSensor STATIC ${AP_InertialSensor})
cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL_AVR_SITL
  AnalogIn
  HAL_AVR_SITL_Class
  RCInput
  RCOutput
  Scheduler
  sitl_barometer
  sitl_compass
  sitl_gps
  sitl_ins
  sitl_optical_flow
  SITL_State
  Storage
  UARTDriver
)

ADD_LIBRARY ( AP_HAL_AVR_SITL STATIC ${AP_HAL_AVR_SITL} )
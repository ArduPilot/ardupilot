cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL_PX4
  AnalogIn
  GPIO
  HAL_PX4_Class
  RCInput
  RCOutput
  Scheduler
  Storage
  UARTDriver
  Util
)

ADD_LIBRARY ( AP_HAL_PX4 STATIC ${AP_HAL_PX4} )
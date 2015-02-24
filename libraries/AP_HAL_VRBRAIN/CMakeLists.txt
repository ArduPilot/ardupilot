cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL_VRBRAIN
  AnalogIn
  GPIO
  HAL_VRBRAIN_Class
  RCInput
  RCOutput
  Scheduler
  Storage
  UARTDriver
  Util
)

ADD_LIBRARY ( AP_HAL_VRBRAIN STATIC ${AP_HAL_VRBRAIN} )
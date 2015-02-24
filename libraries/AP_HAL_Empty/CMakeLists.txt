cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL_Empty
  AnalogIn
  GPIO
  HAL_Empty_Class
  PrivateMember
  RCInput
  RCOutput
  Scheduler
  Semaphores
  SPIDriver
  Storage
  UARTDriver
)

ADD_LIBRARY ( AP_HAL_Empty STATIC ${AP_HAL_Empty} )
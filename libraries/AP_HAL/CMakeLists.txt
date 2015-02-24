cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_HAL
  UARTDriver.cpp
  Util
)

ADD_LIBRARY (AP_HAL STATIC ${AP_HAL})
cmake_minimum_required(VERSION 3.0.2)
project(ArduLibrary)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES) 
SET( AP_InertialSensor
  ${SUB_SOURCES}
)

ADD_LIBRARY( AP_InertialSensor ${AP_InertialSensor} )
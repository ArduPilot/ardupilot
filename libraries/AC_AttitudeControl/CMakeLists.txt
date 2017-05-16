cmake_minimum_required(VERSION 3.0.2)
project(ArduLibrary)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES) 
SET( AC_AttitudeControl
  ${SUB_SOURCES}
)

ADD_LIBRARY ( AC_AttitudeControl ${AC_AttitudeControl} )
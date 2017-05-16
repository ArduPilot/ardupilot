cmake_minimum_required(VERSION 3.0.2)
project(ArduLibrary)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES) 
SET( AP_Baro
  ${SUB_SOURCES}
)


ADD_LIBRARY ( AP_Baro ${AP_Baro} )
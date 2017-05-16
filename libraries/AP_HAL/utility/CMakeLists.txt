cmake_minimum_required(VERSION 3.0.2)
project(ArduLibrary)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES) 
SET( ap_hal_utility
  ${SUB_SOURCES}
)

ADD_LIBRARY ( ap_hal_utility ${ap_hal_utility} )
cmake_minimum_required(VERSION 3.0.2)
project(ArduLibrary)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES) 
SET( AP_HAL
  ${SUB_SOURCES}
)

include_directories ( utility )
add_subdirectory ( utility ) 

ADD_LIBRARY ( AP_HAL ${AP_HAL} )
TARGET_LINK_LIBRARIES ( AP_HAL ap_hal_utility ) 
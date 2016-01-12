# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#

cmake_minimum_required(VERSION 3.0.2)
project(AntennaTracker)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES) 
SET( AntennaTracker
  ${SUB_SOURCES}
)

message( STATUS "Include ArduPilot library directories for AntennaTracker" )
foreach( DIR ${AP_List} ${AP_List_Linux} ${AP_Headers} )
	include_directories( "../libraries/${DIR}" )
endforeach()

# ***************************************
# Build the firmware
# ***************************************
ADD_EXECUTABLE ( AntennaTracker ${AntennaTracker} ${PROJECT_FILE} )

# Native or Linux build
if(LIN_TARGET EQUAL 1)
  target_link_libraries ( AntennaTracker -Wl,--start-group ${AP_List} ${AP_List_Linux} -Wl,--end-group ) 
# PX4/NuttX
else()
  target_link_libraries ( AntennaTracker -Wl,--start-group ${AP_List} -Wl,--end-group ) 
endif()

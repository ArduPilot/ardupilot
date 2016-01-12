# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#
cmake_minimum_required(VERSION 3.0.2)
project(ArduLibrary)

message( STATUS "Include ArduPilot library directories" )
include_directories ( ${AP_List} ${AP_List_Linux} ${AP_Headers} )

message( STATUS "Include sub-directories into build process:" )
foreach( DIR ${AP_List} )
	message( STATUS "  - " ${DIR} )
	add_subdirectory( ${DIR} )
endforeach()
if( ${UNIX} )
	message( STATUS "AP_Library: Unix like environment detected. Include dependent library modules into build process." )
	foreach( DIR ${AP_List_Linux} )
		message( STATUS "  - " ${DIR} )
		add_subdirectory( ${DIR} )
	endforeach()
endif()




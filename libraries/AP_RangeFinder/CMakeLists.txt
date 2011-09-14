set(LIB_NAME AP_RangeFinder)
	
set(${LIB_NAME}_SRCS 
	AP_RangeFinder_MaxsonarXL.cpp
	AP_RangeFinder_SharpGP2Y.cpp
	RangeFinder.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  
	AP_RangeFinder.h
	AP_RangeFinder_MaxsonarXL.h
	AP_RangeFinder_SharpGP2Y.h
	RangeFinder.h
	)
	
include_directories(
 
  -
  #${CMAKE_SOURCE_DIR}/libraries/AP_Math
  #${CMAKE_SOURCE_DIR}/libraries/AP_Common
	#${CMAKE_SOURCE_DIR}/libraries/FastSerial
#	
	)

set(${LIB_NAME}_BOARD mega2560)

generate_arduino_library(${LIB_NAME})

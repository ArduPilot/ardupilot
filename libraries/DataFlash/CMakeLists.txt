set(LIB_NAME DataFlash)
	
set(${LIB_NAME}_SRCS 
		DataFlash.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  
		DataFlash.h
	)
	
include_directories(
 
  -
#  ${CMAKE_SOURCE_DIR}/libraries/AP_Common
	#${CMAKE_SOURCE_DIR}/libraries/FastSerial
#	
	)

set(${LIB_NAME}_BOARD mega2560)

generate_arduino_library(${LIB_NAME})

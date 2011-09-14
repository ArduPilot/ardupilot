set(LIB_NAME APM_BMP085)
	
set(${LIB_NAME}_SRCS 
		APM_BMP085.cpp		
	)  # Firmware sources

set(${LIB_NAME}_HDRS  		
		APM_BMP085.h		
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

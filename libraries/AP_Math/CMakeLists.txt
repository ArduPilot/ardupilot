set(LIB_NAME AP_Math)
	
set(${LIB_NAME}_SRCS 
		
	)  # Firmware sources

set(${LIB_NAME}_HDRS  
		AP_Math.h
		matrix3.h
		vector2.h
		vector3.h
	)
	
include_directories(
 
  -
#  ${CMAKE_SOURCE_DIR}/libraries/AP_Common
	#${CMAKE_SOURCE_DIR}/libraries/FastSerial
#	
	)

set(${LIB_NAME}_BOARD mega2560)

generate_arduino_library(${LIB_NAME})

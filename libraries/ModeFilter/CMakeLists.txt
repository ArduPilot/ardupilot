set(LIB_NAME ModeFilter)
	
set(${LIB_NAME}_SRCS 
		ModeFilter.cpp
		#AP_OpticalFlow_ADNS3080.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  		
		ModeFilter.h
		#AP_OpticalFlow_ADNS3080.h
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

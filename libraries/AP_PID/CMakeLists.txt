set(LIB_NAME AP_PID)
	
set(${LIB_NAME}_SRCS 
		AP_PID.cpp
		#AP_OpticalFlow_ADNS3080.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  		
		AP_PID.h
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

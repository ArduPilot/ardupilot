set(LIB_NAME RC_Channel)
	
set(${LIB_NAME}_SRCS 
		RC_Channel.cpp
		RC_Channel_aux.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  
		RC_Channel.h
		RC_Channel_aux.h
	)
	
include_directories(
 
  -
  ${CMAKE_SOURCE_DIR}/libraries/AP_Common
  ${CMAKE_SOURCE_DIR}/libraries/APM_RC
	#${CMAKE_SOURCE_DIR}/libraries/FastSerial
#	
	)

set(${LIB_NAME}_BOARD mega2560)

generate_arduino_library(${LIB_NAME})

set(LIB_NAME AP_DCM)
	
set(${LIB_NAME}_SRCS 
		AP_DCM.cpp
		AP_DCM_HIL.cpp
		#AP_OpticalFlow_ADNS3080.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  		
		AP_DCM.h
		AP_DCM_HIL.h
	)
	
include_directories(
 
  -
  ${CMAKE_SOURCE_DIR}/libraries/AP_DCM
  #${CMAKE_SOURCE_DIR}/libraries/AP_Common
	#${CMAKE_SOURCE_DIR}/libraries/FastSerial
#	
	)

set(${LIB_NAME}_BOARD mega2560)

generate_arduino_library(${LIB_NAME})

set(LIB_NAME AP_IMU)
	
set(${LIB_NAME}_SRCS 
	AP_IMU_Oilpan.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  
	AP_IMU.h
	AP_IMU_Shim.h
	AP_IMU_Oilpan.h
	IMU.h
	)
	
include_directories(
 
  -
  #${CMAKE_SOURCE_DIR}/libraries/AP_Math
  ${CMAKE_SOURCE_DIR}/libraries/AP_Common
  ${CMAKE_SOURCE_DIR}/libraries/FastSerial
#	
	)

set(${LIB_NAME}_BOARD mega2560)

generate_arduino_library(${LIB_NAME})

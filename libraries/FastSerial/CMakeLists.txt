set(LIB_NAME FastSerial)
	
set(${LIB_NAME}_SRCS 
		BetterStream.cpp		
		FastSerial.cpp
		vprintf.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  		
		BetterStream.h
		FastSerial.h
		ftoa_engine.h
		ntz.h
		xtoa_fast.h
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

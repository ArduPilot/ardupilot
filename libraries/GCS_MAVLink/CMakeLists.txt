set(LIB_NAME GCS_MAVLink)
	
set(${LIB_NAME}_SRCS 
        GCS_MAVLink.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  
      GCS_MAVLink.h
)
	
include_directories(
	#${CMAKE_SOURCE_DIR}/libraries/ 
	${CMAKE_SOURCE_DIR}/libraries/FastSerial
#	
	)

set(${LIB_NAME}_BOARD mega2560)

generate_arduino_library(${LIB_NAME})

set(LIB_NAME AP_Compass)
	
set(${LIB_NAME}_SRCS 
		AP_Compass_HIL.cpp
		AP_Compass_HMC5843.cpp
		Compass.cpp		
	)  # Firmware sources

set(${LIB_NAME}_HDRS  		
		Compass.h
		AP_Compass_HIL.h
		AP_Compass_HMC5843.h
		AP_Compass.h
	)
	

	
include_directories(
 
  -
  ${ARDUINO_LIBRARIES_PATH}/Wire
  #${CMAKE_SOURCE_DIR}/libraries/FastSerial
  #
)
set(${LIB_NAME}_BOARD mega2560)

generate_arduino_library(${LIB_NAME})

set(LIB_NAME AP_ADC)
	
set(${LIB_NAME}_SRCS 
		AP_ADC_HIL.cpp
		AP_ADC_ADS7844.cpp
		AP_ADC.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  		
		AP_ADC_HIL.h
		AP_ADC_ADS7844.h
		AP_ADC.h
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

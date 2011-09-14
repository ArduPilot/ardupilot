set(LIB_NAME AP_GPS)
	
set(${LIB_NAME}_SRCS 
		AP_GPS_406.cpp
		AP_GPS_Auto.cpp
		AP_GPS_HIL.cpp
		AP_GPS_IMU.cpp
		AP_GPS_MTK.cpp
		AP_GPS_MTK16.cpp
		AP_GPS_NMEA.cpp
		AP_GPS_SIRF.cpp
		AP_GPS_UBLOX.cpp
		GPS.cpp
	)  # Firmware sources

set(${LIB_NAME}_HDRS  		
		AP_GPS_406.h
		AP_GPS_Auto.h
		AP_GPS_HIL.h
		AP_GPS_IMU.h
		AP_GPS_MTK.h
		AP_GPS_MTK_Common.h
		AP_GPS_MTK16.h
		AP_GPS_NMEA.h
		AP_GPS_None.h
		AP_GPS_Shim.h
		AP_GPS_SIRF.h
		AP_GPS_UBLOX.h		
		AP_GPS.h
		GPS.h
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

cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Baro
  AP_Baro
  AP_Baro_Backend
  AP_Baro_BMP085
  AP_Baro_HIL
  AP_Baro_MS5611
  AP_Baro_PX4
)

ADD_LIBRARY ( AP_Baro STATIC ${AP_Baro} )
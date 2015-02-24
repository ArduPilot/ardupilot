cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_GPS
  AP_GPS
  AP_GPS_Glitch
  AP_GPS_MTK
  AP_GPS_MTK19
  AP_GPS_PX4
  AP_GPS_SBP
  AP_GPS_SIRF
  AP_GPS_UBLOX
  GPS_Backend
)

ADD_LIBRARY (AP_GPS STATIC ${AP_GPS})
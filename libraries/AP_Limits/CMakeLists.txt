cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Limits
  AP_Limit_Altitude
  AP_Limit_Geofence
  AP_Limit_GPSLock
  AP_Limit_Module
  AP_Limits
)

ADD_LIBRARY (AP_Limits STATIC ${AP_Limits})
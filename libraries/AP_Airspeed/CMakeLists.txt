cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Airspeed
  Airspeed_Calibration
  AP_Airspeed
  AP_Airspeed_analog
  AP_Airspeed_I2C
  AP_Airspeed_I2C_PX4
  AP_Airspeed_PX4
)

ADD_LIBRARY ( AP_Airspeed STATIC ${AP_Airspeed} )
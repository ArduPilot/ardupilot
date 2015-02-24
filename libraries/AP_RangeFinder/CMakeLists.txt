cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_RangeFinder
  AP_RangeFinder_analog
  AP_RangeFinder_MaxsonarI2CXL
  AP_RangeFinder_PulsedLightLRF
  AP_RangeFinder_PX4
  RangeFinder
  RangeFinder_Backend
)

ADD_LIBRARY ( AP_RangeFinder STATIC ${AP_RangeFinder} )
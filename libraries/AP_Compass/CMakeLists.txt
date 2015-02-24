cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Compass
  AP_Compass_AK8963
  AP_Compass_HIL
  AP_Compass_HMC5843
  AP_Compass_PX4
  AP_Compass_VRBRAIN
  Compass
  Compass_learn
)

ADD_LIBRARY ( AP_Compass STATIC ${AP_Compass} )
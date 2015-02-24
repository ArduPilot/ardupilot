cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( APM_Control
  AP_AutoTune
  AP_PitchController
  AP_RollController
  AP_SteerController
  AP_YawController
)

ADD_LIBRARY ( APM_Control STATIC ${APM_Control} )
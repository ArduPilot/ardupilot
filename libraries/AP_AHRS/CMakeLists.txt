cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_AHRS
  AP_AHRS
  AP_AHRS_DCM
  AP_AHRS_NavEKF
)

ADD_LIBRARY ( AP_AHRS STATIC ${AP_AHRS} )
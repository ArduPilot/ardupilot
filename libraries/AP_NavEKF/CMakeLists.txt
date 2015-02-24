cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_NavEKF
  AP_NavEKF
  AP_SmallEKF
)

ADD_LIBRARY ( AP_NavEKF STATIC ${AP_NavEKF} )
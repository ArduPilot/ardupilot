cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_InertialNav
  AP_InertialNav
  AP_InertialNav_NavEKF
)

ADD_LIBRARY ( AP_InertialNav STATIC ${AP_InertialNav} )
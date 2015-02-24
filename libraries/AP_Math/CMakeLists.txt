cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Math 
  AP_Math.cpp
  edc
  location
  matrix3
  polygon
  quaternion
  vector2
  vector3
)

ADD_LIBRARY (AP_Math STATIC ${AP_Math})
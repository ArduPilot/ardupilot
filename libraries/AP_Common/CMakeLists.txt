cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Common
  AP_Common.cpp
  c++
)

ADD_LIBRARY (AP_Common STATIC ${AP_Common})
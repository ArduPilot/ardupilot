cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_OpticalFlow
  AP_OpticalFlow_HIL
  AP_OpticalFlow_PX4
  OpticalFlow
  OpticalFlow_backend  
)

ADD_LIBRARY ( AP_OpticalFlow STATIC ${AP_OpticalFlow} )
cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_Progmem
  AP_Progmem_AVR
)

ADD_LIBRARY ( AP_Progmem STATIC ${AP_Progmem} )
cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_BoardConfig
  AP_BoardConfig
)

ADD_LIBRARY ( AP_BoardConfig STATIC ${AP_BoardConfig} )
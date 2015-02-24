cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( DataFlash
  DataFlash_APM1
  DataFlash_APM2
  DataFlash_Block
  DataFlash_Empty
  DataFlash_File
  DataFlash_SITL
  LogFile
)

ADD_LIBRARY (DataFlash STATIC ${DataFlash})
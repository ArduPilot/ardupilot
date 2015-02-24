cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AC_PID
  AC_HELI_PID
  AC_P
  AC_PID
)

ADD_LIBRARY ( AC_PID STATIC ${AC_PID} )
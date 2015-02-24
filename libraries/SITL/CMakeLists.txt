cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( SITL
  SITL
)

ADD_LIBRARY ( SITL STATIC ${SITL} )
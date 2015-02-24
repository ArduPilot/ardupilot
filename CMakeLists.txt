# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#

cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

message( STATUS "Include cmake modules" )
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")
include(CMakeParseArguments)
include(APMOption)

message( STATUS "Include options file" )
include(options.cmake)

message( STATUS "Add proper definitions" )
apm_option_genflags(BUILD_FLAGS APM_BUILD_FLAGS)
apm_option_genhfile(FILE "config.h")
add_definitions(${APM_BUILD_FLAGS})
message( STATUS "APM_BUILD_FLAGS are: " ${APM_BUILD_FLAGS} )

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

set(OPT_FLAGS "-Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2")
set(WARN_FLAGS "-Wno-unused-function -Wno-unused-parameter")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OPT_FLAGS} ${WARN_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPT_FLAGS} ${WARN_FLAGS} -Wno-reorder")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OPT_FLAGS} ${WARN_FLAGS} -Wl,--relax")

message( STATUS "Walk through the sub directories" )
add_subdirectory (libraries)


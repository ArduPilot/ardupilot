# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#

cmake_minimum_required(VERSION 3.0.2)

###########################################################################
# Add include path' for cmake modules
###########################################################################
message( STATUS "Include cmake modules" )
list(APPEND CMAKE_MODULE_PATH 
  "${CMAKE_SOURCE_DIR}/cmake/modules" 
  "${CMAKE_SOURCE_DIR}/cmake/config" 
  "${CMAKE_SOURCE_DIR}/cmake/toolchains" 
)

###########################################################################
# Generate build flags and config files
# .. if required ..
###########################################################################
include(CMakeParseArguments)
include(APMOption)

message( STATUS "Include options file" )
unset(APM_BUILD_FLAGS_LIST CACHE) # Clear old CACHE values ..
include(options.cmake)            # Read current build options

message( STATUS "Generate definitions (build flags) from cmake option file (or console parameters)" )
apm_option_genflags(BUILD_FLAGS APM_BUILD_FLAGS)
#apm_option_genhfile(FILE "config.h")

###########################################################################
# Define here the toolchain to use
# Currently this just influences which libraries to link together
# E.g. Linux will require AP_HAL_Linux, which is not used for other
# build targets
# Later I may enforce application of toolchain files :D
###########################################################################
message( STATUS "Analyze the definitions for hardware/platform settings" )
apm_find_flag_parameter("${APM_BUILD_FLAGS}" "CONFIG_HAL_BOARD" BOARD_TYPE )

set ( NTX_TARGET 0 ) # HAL_BOARD_PX4
set ( LIN_TARGET 0 ) # HAL_BOARD_LINUX
set ( VRB_TARGET 0 ) # HAL_BOARD_VRBRAIN
set ( FLM_TARGET 0 ) # HAL_BOARD_FLYMAPLE

if( ${BOARD_TYPE} STREQUAL "HAL_BOARD_PX4" OR 
    ${BOARD_TYPE} STREQUAL "HAL_BOARD_VRBRAIN" OR 
    ${BOARD_TYPE} STREQUAL "HAL_BOARD_FLYMAPLE" )

  message( STATUS "Build target is NuttX" )
  set ( NTX_TARGET 1 ) 

# Nothing to do on the toolchain
elseif( ${BOARD_TYPE} STREQUAL "HAL_BOARD_LINUX" )

  message( STATUS "Build target is Linux" )
  set ( LIN_TARGET 1 )
  
else()
  message( FATAL_ERROR "CONFIG_HAL_BOARD not defined" )
  
endif()


###########################################################################
# Add some ArduPilot related build flags (board type, ..)
###########################################################################
# Delete disbaled build flags
apm_delete_disabled_flags("${APM_BUILD_FLAGS}" ACTIVE_BUILD_FLAGS)
unset(APM_BUILD_FLAGS CACHE) 
set(APM_BUILD_FLAGS ${ACTIVE_BUILD_FLAGS}) 

# Print active build flags
message( STATUS "APM_BUILD_FLAGS are:")
foreach(FLAG ${APM_BUILD_FLAGS})
  message( STATUS "  " ${FLAG} )
endforeach()

# Add the definitions globally
add_definitions( ${APM_BUILD_FLAGS} )

###########################################################################
# Project
###########################################################################
project(ArduPilot)

###########################################################################
# Project Module Preparation
###########################################################################
include( git_submodule )

###########################################################################
# Project Configuration
###########################################################################
include( ap-libraries )
include( ap-flags )

###########################################################################
# Check for the presence of some modules
###########################################################################
find_package( i2c-tools )

###########################################################################
# Library module list
###########################################################################
message( STATUS "Include ArduPilot library top directory: ${CMAKE_CURRENT_SOURCE_DIR}/libraries" )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libraries )

#include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/modules/PX4Firmware/src/ )
#include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/modules/PX4Firmware/src/platforms )
#include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/modules/PX4Firmware/src/platforms/posix/include )
#include( ${CMAKE_CURRENT_SOURCE_DIR}/modules/PX4Firmware )
#add_subdirectory( ${CMAKE_CURRENT_SOURCE_DIR}/modules/PX4Firmware )

###########################################################################
# Compiler and linker settings
###########################################################################
set(CMAKE_C_FLAGS           "${CMAKE_C_FLAGS}   ${REQ_FLAGS} ${OPT_FLAGS} ${WRN_FLAGS}")
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} ${REQ_FLAGS} ${OPT_FLAGS} ${WRN_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--relax")

###########################################################################
# Build the library
###########################################################################
message( STATUS "Walk through the sub directories of the library" )
add_subdirectory( libraries )

###########################################################################
# Build the vehicle code
###########################################################################
# Which sketch was defined to be build?
apm_find_flag_parameter("${APM_BUILD_FLAGS}" "SKETCHNAME" SKETCHN )
if( ${SKETCHN} STREQUAL "" )
  message( FATAL_ERROR "NO build target defined" )
else()
  # Finally, build!
  include_directories ( ${SKETCHN} )
  add_subdirectory( ${SKETCHN} )
  message( STATUS "Build target: ${SKETCHN}" )
endif()


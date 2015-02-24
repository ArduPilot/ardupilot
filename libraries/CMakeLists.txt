# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#

cmake_minimum_required(VERSION 2.8)
project(ArduLibrary)

SET( AP_List
  AC_AttitudeControl
  AC_Fence
  AC_PID
  AC_Sprayer
  AC_WPNav
  AP_ADC
  AP_ADC_AnalogSource
  AP_AHRS
  AP_Airspeed
  AP_Arming
  AP_Baro
  AP_BattMonitor
  AP_BoardConfig
  AP_Buffer
  AP_Camera
  AP_Common
  AP_Compass
  AP_Curve
  AP_Declination
  AP_EPM
  AP_Frsky_Telem
  AP_GPS
  AP_HAL
  AP_HAL_AVR
  AP_HAL_AVR_SITL
  AP_HAL_Empty
  AP_HAL_FLYMAPLE
  #AP_HAL_Linux # Commented out because I use MinGW at work
  AP_HAL_PX4
  AP_HAL_VRBRAIN
  AP_InertialNav
  #AP_InertialSensor # Dependency to linux/rtc.h, commented out because I use MinGW at work
  AP_L1_Control
  AP_LandingGear
  #AP_Limits  # Broken class, needs to be fixed by the ArduGuys
  AP_Math
  AP_Menu
  AP_Mission
  AP_Motors
  AP_NavEKF
  AP_Navigation
  AP_Notify
  AP_OpticalFlow
  AP_Parachute
  AP_Param
  AP_PerfMon
  AP_Progmem
  AP_Rally
  AP_RangeFinder
  AP_RCMapper
  AP_Relay
  AP_Scheduler
  AP_SerialManager
  AP_ServoRelayEvents
  AP_SpdHgtControl
  AP_TECS
  AP_Terrain
  AP_Vehicle
  APM_Control
  APM_OBC
  APM_PI
  DataFlash
  Filter
  #GCS_Console # Broken class, needs to be fixed by the ArduGuys
  GCS_MAVLink
  RC_Channel
  SITL
  StorageManager
  AP_InertialSensor # delete this later
)

message( STATUS "Include ArduPilot library directories" )
include_directories (${AP_List})

message( STATUS "Include sub-directories into build process:" )
foreach( DIR ${AP_List} )
	message( STATUS "  - " ${DIR} )
	add_subdirectory( ${DIR} )
endforeach()

message( STATUS "Look for all the build libraries and add them to the track list" )
foreach( DIR ${AP_List} )
	list( APPEND LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/${DIR}" )
endforeach()
list( REMOVE_DUPLICATES LIB_DIRS )
foreach( DIR ${AP_List} )
	FIND_LIBRARY( LIB LIB_DIRS )
	list( APPEND LIBS ${LIB} )
endforeach()
list( REMOVE_DUPLICATES LIBS )



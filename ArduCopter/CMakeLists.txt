#=============================================================================#
# Author: Niklaa Goddemeier & Sebastian Rohde                                 #
# Date:   04.09.2011                                                          #
#=============================================================================#

set(CMAKE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/../")

set(CMAKE_MODULE_PATH    ${CMAKE_SOURCE_DIR}/cmake/modules)  # CMake module search path
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Arduino.cmake) # Arduino Toolchain
#include(ArduinoProcessing)

set (CMAKE_CXX_SOURCE_FILE_EXTENSIONS pde)


message(STATUS "DIR: ${CMAKE_SOURCE_DIR}")

cmake_minimum_required(VERSION 2.8)
#====================================================================#
#  Setup Project                                                     #
#====================================================================#
project(ArduCopter C CXX)

find_package(Arduino 22 REQUIRED)

if (NOT DEFINED BOARD)
        message(STATUS "board not defined, assuming mega, use cmake -DBOARD=mega2560 , etc. to specify")
        set(BOARD "mega")
endif()
message(STATUS "Board configured as: ${BOARD}")

add_subdirectory(../libraries "${CMAKE_CURRENT_BINARY_DIR}/libs")

#add_subdirectory(${CMAKE_SOURCE_DIR}/ArduCopter)
#add_subdirectory(testtool)

PRINT_BOARD_SETTINGS(${BOARD})



#=============================================================================#
# Author: Niklas Goddemeier & Sebastian Rohde                                 #
# Date:   04.09.2011                                                          #
#=============================================================================#


#====================================================================#
#  Settings                                                          #
#====================================================================#
set(FIRMWARE_NAME arducopter)

set(${FIRMWARE_NAME}_BOARD ${BOARD})               # Arduino Target board

set(${FIRMWARE_NAME}_SKETCHES
    ArduCopter.pde
	Attitude.pde
	Camera.pde
	commands.pde
	commands_logic.pde
	commands_process.pde
	control_modes.pde
	events.pde
	flip.pde
	GCS.pde
	GCS_Ardupilot.pde
	#GCS_IMU_output.pde
	GCS_Jason_text.pde
	GCS_Mavlink.pde
	GCS_Standard.pde
	GCS_Xplane.pde
	heli.pde
	HIL_Xplane.pde
	leds.pde
	Log.pde
	motors_hexa.pde
	motors_octa.pde
	motors_octa_quad.pde
	motors_quad.pde
	motors_tri.pde
	motors_y6.pde
	navigation.pde
	planner.pde
	radio.pde
	read_commands.pde
	sensors.pde
	setup.pde
	system.pde
	test.pde
	)  # Firmware sketches
	
#create dummy sourcefile
file(WRITE ${FIRMWARE_NAME}.cpp "// Do not edit")
	
set(${FIRMWARE_NAME}_SRCS
		#test.cpp
		${FIRMWARE_NAME}.cpp
	)  # Firmware sources
		
set(${FIRMWARE_NAME}_HDRS 	
	APM_Config.h
	APM_Config_mavlink_hil.h
	APM_Config_xplane.h
	config.h
	defines.h
	GCS.h
	HIL.h
	Mavlink_Common.h
	Parameters.h
	)  # Firmware sources

set(${FIRMWARE_NAME}_LIBS
	DataFlash
	AP_Math
	PID
        RC_Channel
	AP_OpticalFlow
	ModeFilter	
        memcheck
	#AP_PID
	APM_PI
	#APO
        FastSerial
        AP_Common
        GCS_MAVLink
        AP_GPS
        APM_RC
        AP_DCM
        AP_ADC
        AP_Compass
        AP_IMU
        AP_RangeFinder
        APM_BMP085
        c
        m
)
	
#${CONSOLE_PORT}
set(${FIRMWARE_NAME}_PORT COM2)            # Serial upload port
set(${FIRMWARE_NAME}_SERIAL putty -serial COM2 -sercfg 57600,8,n,1,X )  # Serial terminal cmd
	
include_directories(
${CMAKE_SOURCE_DIR}/libraries/DataFlash
${CMAKE_SOURCE_DIR}/libraries/AP_Math
${CMAKE_SOURCE_DIR}/libraries/PID
${CMAKE_SOURCE_DIR}/libraries/AP_Common
${CMAKE_SOURCE_DIR}/libraries/RC_Channel
${CMAKE_SOURCE_DIR}/libraries/AP_OpticalFlow
${CMAKE_SOURCE_DIR}/libraries/ModeFilter
${CMAKE_SOURCE_DIR}/libraries/memcheck
#${CMAKE_SOURCE_DIR}/libraries/AP_PID
${CMAKE_SOURCE_DIR}/libraries/APM_PI
${CMAKE_SOURCE_DIR}/libraries/FastSerial
${CMAKE_SOURCE_DIR}/libraries/AP_Compass
${CMAKE_SOURCE_DIR}/libraries/AP_RangeFinder
${CMAKE_SOURCE_DIR}/libraries/AP_GPS
${CMAKE_SOURCE_DIR}/libraries/AP_IMU
${CMAKE_SOURCE_DIR}/libraries/AP_ADC
${CMAKE_SOURCE_DIR}/libraries/AP_DCM
${CMAKE_SOURCE_DIR}/libraries/APM_RC
${CMAKE_SOURCE_DIR}/libraries/GCS_MAVLink
${CMAKE_SOURCE_DIR}/libraries/APM_BMP085
)
#====================================================================#
#  Target generation                                                 #
#====================================================================#
generate_arduino_firmware(${FIRMWARE_NAME})

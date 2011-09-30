cmake_minimum_required(VERSION 2.6)
set(CMAKE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/../")
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Arduino.cmake) # Arduino Toolchain
set(CMAKE_MODULE_PATH    ${CMAKE_SOURCE_DIR}/cmake/modules)  # CMake module search path

string(REGEX REPLACE ".*/" "" PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR})
project(${PROJECT_NAME} C CXX)

set (CMAKE_CXX_SOURCE_FILE_EXTENSIONS pde)

find_package(Arduino 22 REQUIRED)

if (NOT DEFINED BOARD)
	message(STATUS "board not defined, assuming mega, use cmake -DBOARD=mega2560 , etc. to specify")
	set(BOARD "mega")
endif()
message(STATUS "Board configured as: ${BOARD}")

# need to configure based on host operating system
set(${PROJECT_NAME}_PORT COM2)
set(${PROJECT_NAME}_SERIAL putty -serial COM2 -sercfg 57600,8,n,1,X )
	
include_directories(
${ARDUINO_LIBRARIES_PATH}/Wire
${CMAKE_SOURCE_DIR}/libraries/APO
${CMAKE_SOURCE_DIR}/libraries/AP_Common
${CMAKE_SOURCE_DIR}/libraries/FastSerial
${CMAKE_SOURCE_DIR}/libraries/ModeFilter
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

add_subdirectory(../libraries "${CMAKE_CURRENT_BINARY_DIR}/libs")

set(${PROJECT_NAME}_BOARD ${BOARD})
file(GLOB ${PROJECT_NAME}_SKETCHES *.pde)
file(GLOB ${PROJECT_NAME}_SRCS *.cpp)
file(GLOB ${PROJECT_NAME}_HDRS *.h)
set(${PROJECT_NAME}_LIBS
	c
    m
    APO
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
	ModeFilter
)

generate_arduino_firmware(${PROJECT_NAME})

install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex 
	DESTINATION bin
	)

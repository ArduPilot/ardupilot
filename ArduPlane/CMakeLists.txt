set(CMAKE_TOOLCHAIN_FILE ../cmake/ArduinoToolchain.cmake) # Arduino Toolchain

cmake_minimum_required(VERSION 2.8)

project(ArduPlane C CXX)

set(PROJECT_VERSION_MAJOR "2")
set(PROJECT_VERSION_MINOR "3")
set(PROJECT_VERSION_PATCH "3")

set(PROJECT_DESCRIPTION "ArduPilotMega based Airplane Autopilot.")

# macro path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../cmake/modules")
include(CMakeParseArguments)
include(APMOption)

# options
include(options.cmake)
include_directories(${CMAKE_BINARY_DIR})
add_definitions(-DUSE_CMAKE_APM_CONFIG)
apm_option_generate_config(FILE "APM_Config_cmake.h" BUILD_FLAGS APM_BUILD_FLAGS)
add_definitions(${APM_BUILD_FLAGS})
message(STATUS "build flags: ${APM_BUILD_FLAGS}")
#configure_file(APM_Config2.h.cmake APM_Config2.h)

# disallow in-source build
include(MacroEnsureOutOfSourceBuild)
macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build.
Please create a separate build directory and run 'cmake /path/to/${PROJECT_NAME} [options]' there.")

# built variables
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
set(FIRMWARE_NAME "${PROJECT_NAME}-${CONFIG_APM_HARDWARE}-${APM_PROCESSOR}-${HIL_MODE}")

# modify flags from default toolchain flags
set(APM_OPT_FLAGS "-Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${APM_OPT_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${APM_OPT_FLAGS} -Wno-reorder")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${APM_OPT_FLAGS} -Wl,--relax")

# build apm project
set(ARDUINO_EXTRA_LIBRARIES_PATH ${CMAKE_SOURCE_DIR}/../libraries)
set(${FIRMWARE_NAME}_SKETCH ${CMAKE_SOURCE_DIR}/../${PROJECT_NAME})
set(${FIRMWARE_NAME}_BOARD ${APM_PROCESSOR})
set(${FIRMWARE_NAME}_PORT ${APM_PROGRAMMING_PORT})
set(${FIRMWARE_NAME}_SRCS ${CMAKE_BINARY_DIR}/APM_Config_cmake.h)
generate_arduino_firmware(${FIRMWARE_NAME})
install(FILES ${CMAKE_BINARY_DIR}/${FIRMWARE_NAME}.hex DESTINATION "/")

# cpack
include(APMCPackConfig)

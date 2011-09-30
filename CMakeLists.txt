cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH    ${CMAKE_SOURCE_DIR}/cmake/modules)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Arduino.cmake)

# modify flags from default toolchain flags
set(APM_OPT_FLAGS "-Os -Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2 -Wno-reorder")

set(ARDUINO_C_FLAGS "${APM_OPT_FLAGS} -mcall-prologues -ffunction-sections -fdata-sections")
set(ARDUINO_CXX_FLAGS "${APM_OPT_FLAGS} -mcall-prologues -ffunction-sections -fdata-sections -fno-exceptions")
set(ARDUINO_LINKER_FLAGS "${APM_OPT_FLAGS} -Wl,--gc-sections")



project(ArduPilotMega C CXX)

# set default cmake build type to RelWithDebInfo (None Debug Release RelWithDebInfo MinSizeRel)
if( NOT DEFINED CMAKE_BUILD_TYPE )
	set( CMAKE_BUILD_TYPE "RelWithDebInfo" )
endif()


find_package(Arduino 22 REQUIRED)

# determine board being used
if (NOT DEFINED BOARD)
	message(STATUS "board not defined, assuming mega, use cmake -DBOARD=mega2560 , etc. to specify")
	set(BOARD "mega")
endif()
message(STATUS "Board configured as: ${BOARD}")

set (CMAKE_CXX_SOURCE_FILE_EXTENSIONS pde)

# apo
#set(apo_ASFLAGS "-assembler-with-cpp")
set(apo_BOARD ${BOARD})
set(apo_SRCS
	apo/apo.cpp
	)
generate_arduino_firmware(apo)

# ArduRover
set(ArduRover_BOARD ${BOARD})
set(ArduRover_SRCS
	ArduRover/ArduRover.pde
	)
#generate_arduino_firmware(ArduRover)

# ArduBoat
set(ArduRover_BOARD ${BOARD})
set(ArduRover_SRCS
	ArduRover/ArduBoat.pde
	)
#generate_arduino_firmware(ArduBoat)

# ArduPlane
set(ArduPlane ${BOARD})
set(ArduPlane_SRCS
	)
#generate_arduino_firmware(ArduPlane)

# ArduCopter
set(ArduCopter ${BOARD})
set(ArduCopter_SRCS
	)
#generate_arduino_firmware(ArduCopter)

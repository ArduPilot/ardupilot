cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH    ${CMAKE_SOURCE_DIR}/cmake/modules)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Arduino.cmake)

# modify flags from default toolchain flags
set(APM_OPT_FLAGS "-Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2")

set(ARDUINO_C_FLAGS "${APM_OPT_FLAGS} -mcall-prologues -ffunction-sections -fdata-sections")
set(ARDUINO_CXX_FLAGS "${APM_OPT_FLAGS} -mcall-prologues -ffunction-sections -fdata-sections -fno-exceptions -Wno-reorder")
set(ARDUINO_LINKER_FLAGS "-lc -lm ${APM_OPT_FLAGS} -Wl,--gc-sections")

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

# standard project setup
function(apo_project PROJECT_NAME) 
	message(STATUS "creating apo project ${PROJECT_NAME}")
	set(${PROJECT_NAME}_AFLAGS "-assembler-with-cpp")
	set(${PROJECT_NAME}_BOARD ${BOARD})
	set(${PROJECT_NAME}_SRCS ${PROJECT_NAME}/${PROJECT_NAME}.cpp)
	set(${PROJECT_NAME}_LIBS c)
	generate_arduino_firmware(${PROJECT_NAME})
endfunction()

# apo projects
set(apo_BOARD ${BOARD})
set(apo_AFLAGS "-assembler-with-cpp")
set(apo_SRCS apo/apo.cpp)
set(apo_LIBS c)
generate_arduino_firmware(apo)
#apo_project(apo)
#apo_project(ArduBoat)

# ArduPlane
set(ArduPlane_BOARD ${BOARD})
set(ArduPlane_SRCS
	)
#generate_arduino_firmware(ArduPlane)

# ArduCopter
set(ArduCopter_BOARD ${BOARD})
set(ArduCopter_SRCS
	)
#generate_arduino_firmware(ArduCopter)

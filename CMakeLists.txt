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

# set these for release
set(APPLICATION_VERSION_MAJOR "1")
set(APPLICATION_VERSION_MINOR "2")
set(APPLICATION_VERSION_PATCH "0")

# dependencies
find_package(Arduino 22 REQUIRED)

# cmake settigns
set(APPLICATION_NAME ${PROJECT_NAME})
set(APPLICATION_VERSION "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}.${APPLICATION_VERSION_PATCH}")

# macros
include(MacroEnsureOutOfSourceBuild)

# disallow in-source build
macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build.
Please create a separate build directory and run 'cmake /path/to/${PROJECT_NAME} [options]' there.")

# options
if (NOT DEFINED BOARD)
	message(STATUS "please define the board type (for example: cmake -DBOARD=mega, assuming mega")
	set(BOARD "mega")
endif()

if (NOT DEFINED PORT)
	message(STATUS "please define the upload port (for example: cmake
	-DPORT=/dev/ttyUSB0, assuming /dev/ttyUSB0")
	set(PORT "/dev/ttyUSB0")
endif()

# cpack settings
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A universal autopilot system for the ArduPilotMega board.")
set(CPACK_PACKAGE_VENDOR "DIYDRONES")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "james.goppert@gmail.com")
set(CPACK_PACKAGE_CONTACT "james.goppert@gmail.com")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.txt")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/COPYING.txt")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.txt")
set(CPACK_PACKAGE_VERSION_MAJOR "${APPLICATION_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${APPLICATION_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${APPLICATION_VERSION_PATCH}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
set(CPACK_SET_DESTDIR TRUE)
set(CPACK_SOURCE_IGNORE_FILES ${CPACK_SOURCE_IGNORE_FILES}
	/.git/;/build/;~$;.*\\\\.bin$;.*\\\\.swp$)
set(CPACK_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
set(CPACK_SOURCE_GENERATOR "ZIP")
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_NAME "${APPLICATION_NAME}_${BOARD}")
include(CPack)

find_package(Arduino 22 REQUIRED)

# determine board being used
if (NOT DEFINED BOARD)
	message(STATUS "board not defined, assuming mega, use cmake -DBOARD=mega2560 , etc. to specify")
	set(BOARD "mega")
endif()
message(STATUS "Board configured as: ${BOARD}")

set (CMAKE_CXX_SOURCE_FILE_EXTENSIONS pde)

# standard apm project setup
macro(apm_project PROJECT_NAME BOARD SRCS) 
	message(STATUS "creating apo project ${PROJECT_NAME}")
	set(${PROJECT_NAME}_BOARD ${BOARD})
	set(${PROJECT_NAME}_PORT ${PORT})
	set(${PROJECT_NAME}_SRCS ${SRCS})
	set(${PROJECT_NAME}_LIBS m c)
	generate_arduino_firmware(${PROJECT_NAME})
	set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
	install(FILES
		${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex 
		DESTINATION bin
	)
endmacro()

# projects
apm_project(apo ${BOARD} apo/apo.cpp)
apm_project(ArduRover ${BOARD} ArduRover/ArduRover.cpp)
apm_project(ArduBoat ${BOARD} ArduBoat/ArduBoat.cpp)
apm_project(ArduPlane ${BOARD} ArduPlane/ArduPlane.cpp)
apm_project(ArduCopter ${BOARD} ArduCopter/ArduCopter.cpp)

set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake) # Arduino Toolchain

cmake_minimum_required(VERSION 2.8)

project(ArduPilotMega C CXX)

# set these for release
set(PROJECT_VERSION_MAJOR "2")
set(PROJECT_VERSION_MINOR "3")
set(PROJECT_VERSION_PATCH "3")
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")

# options
option(BUILD_2560 "Build 2560 firmware?" ON)
option(BUILD_1280 "Build 1280 firmware?" OFF)

if (NOT DEFINED PORT)
	message(WARNING "please define the upload port (for example: cmake
	-DPORT=/dev/ttyUSB0, assuming /dev/ttyUSB0")
	set(PORT "/dev/ttyUSB0")
endif()

# macro path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# disallow in-source build
include(MacroEnsureOutOfSourceBuild)
macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build.
Please create a separate build directory and run 'cmake /path/to/${PROJECT_NAME} [options]' there.")

# modify flags from default toolchain flags
set(APM_OPT_FLAGS "-Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${APM_OPT_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${APM_OPT_FLAGS} -Wno-reorder")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${APM_OPT_FLAGS} -Wl,--relax")

# projects
set(PROJECT_LIST
    ArduPlane
    ArduCopter
    #apo
    #ArduBoat
    #ArduRover
    )

# macro for building firmware for all projects
macro(build_apm_firmware BOARD)
    foreach(PROJECT ${PROJECT_LIST})
        set(${PROJECT}_SKETCH ${CMAKE_SOURCE_DIR}/${PROJECT})
        set(${PROJECT}_BOARD ${BOARD})
        set(${PROJECT}_PORT ${PORT})
        generate_arduino_firmware(${PROJECT})
    endforeach()
endmacro()

# build firmware based on options
if (BUILD_2560)
    build_apm_firmware(mega2560)
endif()

if (BUILD_1280)
    build_apm_firmware(mega)
endif()

# packaging settings
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

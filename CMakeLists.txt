set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake) # Arduino Toolchain

cmake_minimum_required(VERSION 2.8)

project(ArduPilotMega C CXX)

# set these for release
set(PROJECT_VERSION_MAJOR "2")
set(PROJECT_VERSION_MINOR "3")
set(PROJECT_VERSION_PATCH "3")
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")


# macro path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# disallow in-source build
include(MacroEnsureOutOfSourceBuild)
macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build.
Please create a separate build directory and run 'cmake /path/to/${PROJECT_NAME} [options]' there.")

# macros
include(CMakeParseArguments)
include(APMOption)

# options
apm_option("APM_PROGRAMMING_PORT" TYPE "STRING"
    DESCRIPTION "Programming upload port?"
    DEFAULT "/dev/ttyUSB0")

apm_option("APM_BOARD" TYPE "COMBO"
    DESCRIPTION "ArduPilotMega board?" 
    DEFAULT "mega2560"
    OPTIONS "mega" "mega2560")

apm_option("APM_PROJECT" TYPE "COMBO"
    DESCRIPTION "ArduPilotMega project to build?"
    DEFAULT "ArduPlane"
    OPTIONS "ArduPlane" "ArduCopter")

if(APM_PROJECT STREQUAL "ArduPlane")
    include(cmake/options-ArduPlane.cmake)
endif()

# modify flags from default toolchain flags
set(APM_OPT_FLAGS "-Wformat -Wall -Wshadow -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${APM_OPT_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${APM_OPT_FLAGS} -Wno-reorder")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${APM_OPT_FLAGS} -Wl,--relax")

# build apm project
set(${APM_PROJECT}_SKETCH ${CMAKE_SOURCE_DIR}/${APM_PROJECT})
set(${APM_PROJECT}_BOARD ${APM_BOARD})
set(${APM_PROJECT}_PORT ${APM_PROGRAMMING_PORT})
generate_arduino_firmware(${APM_PROJECT})

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
set(CPACK_PACKAGE_NAME "${APM_PROJECT}_${BOARD}_${HIL_MODE}")
include(CPack)

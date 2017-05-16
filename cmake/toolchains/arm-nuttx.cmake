# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#
include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME       EXTERNAL)
set(CMAKE_SYSTEM_PROCESSOR  arm)

set(EXTERNAL_CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(EXTERNAL_CMAKE_CXX_COMPILER arm-none-eabi-g++)

CMAKE_FORCE_C_COMPILER(${EXTERNAL_CMAKE_C_COMPILER} GNU)
CMAKE_FORCE_CXX_COMPILER(${EXTERNAL_CMAKE_CXX_COMPILER} GNU)

set(NO_PTHREAD YES)
set(BUILD_TO_LIB YES)

set(FLAGS_COMMON -mcpu=cortex-m4
                 -mthumb
                 -march=armv7e-m
                 -mfpu=fpv4-sp-d16
                 -mfloat-abi=hard
                 -D__NUTTX__
                 -D__ARM__
                 -Os
                 -fno-strict-aliasing
                 -fno-strength-reduce
                 -fomit-frame-pointer)

foreach(FLAG ${FLAGS_COMMON})
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAG}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")
endforeach()

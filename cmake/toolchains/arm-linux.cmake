# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#
include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME       Linux)
set(CMAKE_SYSTEM_PROCESSOR  arm)

set(EXTERNAL_CMAKE_C_COMPILER   arm-linux-gnueabihf-gcc)
set(EXTERNAL_CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

CMAKE_FORCE_C_COMPILER(${EXTERNAL_CMAKE_C_COMPILER} GNU)
CMAKE_FORCE_CXX_COMPILER(${EXTERNAL_CMAKE_CXX_COMPILER} GNU)

set(FLAGS_COMMON -lrt
                 -lm
                 -lpthread
                 -D__LINUX__
                 -D__ARM__)

foreach(FLAG ${FLAGS_COMMON})
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAG}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")
endforeach()

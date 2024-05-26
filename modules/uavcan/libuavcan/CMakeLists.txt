#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

cmake_minimum_required(VERSION 2.8)

if(DEFINED CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Debug Release RelWithDebInfo MinSizeRel")
else()
   set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Debug Release RelWithDebInfo MinSizeRel")
endif()

# Detecting whether we need to add debug targets
string(TOLOWER "${CMAKE_BUILD_TYPE}" build_type_lower)
if (build_type_lower STREQUAL "debug")
    set(DEBUG_BUILD 1)
    message(STATUS "Debug build")
else ()
    set(DEBUG_BUILD 0)
endif ()

project(libuavcan)

find_package(PythonInterp)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(COMPILER_IS_GCC_COMPATIBLE 1)
else ()
    set(COMPILER_IS_GCC_COMPATIBLE 0)
endif ()

#
# DSDL compiler invocation
# Probably output files should be saved into CMake output dir?
#
execute_process(COMMAND ${PYTHON_EXECUTABLE} setup.py build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dsdl_compiler OUTPUT_QUIET)
set(DSDLC_INPUTS "test/dsdl_test/root_ns_a" "test/dsdl_test/root_ns_b" "${CMAKE_CURRENT_SOURCE_DIR}/../dsdl/uavcan")
set(DSDLC_OUTPUT "include/dsdlc_generated")

set(DSDLC_INPUT_FILES "")
foreach(DSDLC_INPUT ${DSDLC_INPUTS})
    file(GLOB_RECURSE DSDLC_NEW_INPUT_FILES ${CMAKE_CURRENT_SOURCE_DIR} "${DSDLC_INPUT}/*.uavcan")
    set(DSDLC_INPUT_FILES ${DSDLC_INPUT_FILES} ${DSDLC_NEW_INPUT_FILES})
endforeach(DSDLC_INPUT)
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/libuavcan_dsdlc_run.stamp
                   COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/dsdl_compiler/libuavcan_dsdlc ${DSDLC_INPUTS} -O${DSDLC_OUTPUT}
                   COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_BINARY_DIR}/libuavcan_dsdlc_run.stamp
                   DEPENDS ${DSDLC_INPUT_FILES}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   COMMENT "Running dsdl compiler")
add_custom_target(libuavcan_dsdlc DEPENDS ${CMAKE_BINARY_DIR}/libuavcan_dsdlc_run.stamp)
include_directories(${DSDLC_OUTPUT})

#
# Compiler flags
#
if (COMPILER_IS_GCC_COMPATIBLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wundef")
    message(STATUS "Using C++11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif ()

if (DEBUG_BUILD)
    add_definitions(-DUAVCAN_DEBUG=1)
endif ()

include_directories(include)

#
# libuavcan
#
file(GLOB_RECURSE LIBUAVCAN_CXX_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/*.cpp")
add_library(uavcan STATIC ${LIBUAVCAN_CXX_FILES})
add_dependencies(uavcan libuavcan_dsdlc)

install(TARGETS uavcan                            DESTINATION lib)
install(DIRECTORY include/uavcan                  DESTINATION include)
install(DIRECTORY include/dsdlc_generated/uavcan  DESTINATION include)  # Generated and lib's .hpp
install(CODE "execute_process(COMMAND ${PYTHON_EXECUTABLE} setup.py install --record installed_files.log
                              WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dsdl_compiler)")

#
# Tests and static analysis - only for debug builds
#
function(add_libuavcan_test name library flags) # Adds GTest executable and creates target to execute it every build
    find_package(Threads REQUIRED)

    file(GLOB_RECURSE TEST_CXX_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "test/*.cpp")
    add_executable(${name} ${TEST_CXX_FILES})
    add_dependencies(${name} ${library})

    if (flags)
        set_target_properties(${name} PROPERTIES COMPILE_FLAGS ${flags})
    endif ()

    target_link_libraries(${name} gmock_main)
    target_link_libraries(${name} ${library})
    if (${UAVCAN_PLATFORM} STREQUAL "linux")
        target_link_libraries(${name} rt)
    endif()

    # Tests run automatically upon successful build
    # If failing tests need to be investigated with debugger, use 'make --ignore-errors'
    if (CONTINUOUS_INTEGRATION_BUILD)
        # Don't redirect test output, and don't run tests suffixed with "RealTime"
        add_test(NAME ${name}
                 COMMAND ${name} --gtest_filter=-*RealTime
                 WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    else ()
        add_test(NAME ${name}
             COMMAND ${name} 1>"${name}.log" 2>&1
             WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endif()
endfunction()

if (DEBUG_BUILD)
    message(STATUS "Debug build (note: requires gtest)")

    if (COMPILER_IS_GCC_COMPATIBLE)
        # No such thing as too many warnings
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic -Wfloat-equal -Wconversion")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wsign-conversion -Wcast-align -Wmissing-declarations -Wlogical-op")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdouble-promotion -Wswitch-enum -Wtype-limits -Wno-error=array-bounds")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wzero-as-null-pointer-constant -Wnon-virtual-dtor")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual -Wsign-promo -Wold-style-cast")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations")
        set(optim_flags "-O3 -DNDEBUG -g0")
    else ()
        message(STATUS "Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
        message(FATAL_ERROR "This compiler cannot be used to build tests; use release build instead.")
    endif ()

    # Additional flavours of the library

    add_library(uavcan_optim STATIC ${LIBUAVCAN_CXX_FILES})
    set_target_properties(uavcan_optim PROPERTIES COMPILE_FLAGS ${optim_flags})
    add_dependencies(uavcan_optim libuavcan_dsdlc)

    if (GTEST_FOUND)
        message(STATUS "GTest found, tests will be built and run.")
        add_libuavcan_test(libuavcan_test       uavcan       "")                 # Default
        add_libuavcan_test(libuavcan_test_optim uavcan_optim "${optim_flags}")   # Max optimization
    else (GTEST_FOUND)
        message(STATUS "GTest was not found, tests will not be built")
    endif (GTEST_FOUND)
else ()
    message(STATUS "Release build type: " ${CMAKE_BUILD_TYPE})
endif ()

# vim: set et ft=cmake fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :
